"""
arms_ui_node — OpenCV overlay display

Subscribes to:
  /arms/image_raw      sensor_msgs/Image
  /arms/detections     arms_msgs/DetectionArray
  /arms/mission_state  arms_msgs/MissionState
  /arms/control_debug  geometry_msgs/Vector3
  /arms/command        sensor_msgs/Joy   (buttons[2]: 0=AUTO, 1=MANUAL)

오버레이(바운딩박스/십자선/상태/ROI 등)는 AUTO 모드에서만 그린다.
MANUAL 모드(buttons[2]==1)에서는 원본 영상만 표시한다.
"""

import os
import shutil
import subprocess
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Image, Joy
from std_msgs.msg import Float32MultiArray

from arms_msgs.msg import DetectionArray, MissionState
from geometry_msgs.msg import Vector3

# /arms/command Joy 버튼 인덱스 (arms_command_uart_node.cpp / arms_control_node.cpp 와 일치)
#   buttons[0]=kill, [1]=arm, [2]=mode, [3]=launch
KILL_BUTTON_IDX = 0
MODE_BUTTON_IDX = 2

STATE_COLORS = {
    "IDLE":   (128, 128, 128),
    "SEARCH": (0,   200, 255),
    "LOCK":   (0,   140, 255),
    "BOOST":  (0,   110, 255),
    "TRACK":  (0,   0,   220),
    "FIRE":   (0,   0,   220),
    "RTL":    (255, 180, 0),
}


class ArmsUINode(Node):
    def __init__(self):
        super().__init__("arms_ui_node")
        self._bridge = CvBridge()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # PiP 크기 (프레임 폭 대비 ROI 표시 폭 비율)
        self.declare_parameter("ui.roi_pip_frac", 0.25)
        # ui.cv_debug: 검출 진단 패널(오른쪽에 HSV 미리보기 + 성능 그래프)을 붙일지.
        #   True 면 /arms/hsv_debug_image 를 구독한다. False(실기체 기본) → 메인 화면만.
        #   hsv_debug 를 구독조차 안 하므로 detection 이 발행을 통째로 스킵(구독자 gated) →
        #   컨테이너→호스트 대용량 UDP 발행이 사라져 검출 발행률이 카메라 fps 를 그대로 따라감.
        self.declare_parameter("ui.cv_debug", False)
        self.declare_parameter("ui.diagnostics_history", 180)
        self._cv_debug = bool(self.get_parameter("ui.cv_debug").value)

        # ui.debug: 메인 화면 디버그 오버레이(err/cmd 숫자 텍스트, YOLO/HSV 상태 텍스트,
        #   오차·명령 화살표)를 그릴지. 기본 False → 실기체처럼 메인 화면만.
        #   표적 bbox 는 이 값과 무관하게 항상 그린다(아래 _draw_overlay).
        #   arms.launch.py=False(메인만), arms_replay.launch.py=True(디버그) 로 준다.
        self.declare_parameter("ui.debug", False)
        self._debug = bool(self.get_parameter("ui.debug").value)

        # 모드 전환 효과음 (MP3). 기본값 = 패키지 share/sounds 설치 경로.
        try:
            _snd_dir = os.path.join(get_package_share_directory("arms_ui"), "sounds")
        except Exception:
            _snd_dir = ""
        self.declare_parameter("ui.sound_manual", os.path.join(_snd_dir, "manual.mp3"))
        self.declare_parameter("ui.sound_auto", os.path.join(_snd_dir, "auto.mp3"))
        # 상태 변경 효과음: 수동=arm/disarm, 자동=idle/search
        self.declare_parameter("ui.sound_arm", os.path.join(_snd_dir, "arm.mp3"))
        self.declare_parameter("ui.sound_disarm", os.path.join(_snd_dir, "disarm.mp3"))
        self.declare_parameter("ui.sound_search", os.path.join(_snd_dir, "search.mp3"))
        self.declare_parameter("ui.sound_idle", os.path.join(_snd_dir, "idle.mp3"))
        # 킬 스위치 ON(engage) 경고음
        self.declare_parameter("ui.sound_kill", os.path.join(_snd_dir, "kill.mp3"))
        # ELRS 텔레메트리 연결/끊김 효과음
        self.declare_parameter("ui.sound_telm_connected", os.path.join(_snd_dir, "telm_connected.mp3"))
        self.declare_parameter("ui.sound_telm_lost", os.path.join(_snd_dir, "telm_lost.mp3"))
        self._player = shutil.which("ffplay")
        if self._player is None:
            self.get_logger().warn("ffplay 없음 — 효과음이 재생되지 않습니다.")


        # ── 탐지 비프음 (mp3 없이 ffplay lavfi 사인파로 생성) ──────────────
        #   SEARCH: 느린 간격, LOCK/TRACK/FIRE: 빠른 간격으로 삑삑거린다.
        #   IDLE/RTL 등에선 울리지 않고, 자동(AUTO) 모드에서만 동작한다.
        self.declare_parameter("ui.beep_enabled", True)
        self.declare_parameter("ui.beep_search_interval", 0.8)   # SEARCH 비프 간격[s]
        self.declare_parameter("ui.beep_lock_interval", 0.18)    # LOCK 이후 비프 간격[s]
        self.declare_parameter("ui.beep_freq_hz", 2000.0)        # 비프 주파수[Hz]
        self.declare_parameter("ui.beep_duration", 0.06)         # 비프 길이[s]
        self.declare_parameter("ui.beep_volume", 0.6)            # 비프 음량(0~1)
        self._last_beep_t = 0.0                                  # 마지막 비프 시각(monotonic)
        self._lock_tone_proc = None                             # LOCK 이상: 끊기지 않는 연속 톤 프로세스
        self._last_image_t = 0.0                                # 마지막 영상 프레임 수신 시각(monotonic)

        self.create_subscription(Image, "/arms/image_raw", self._cb_image, best_effort_qos)
        self.create_subscription(DetectionArray, "/arms/detections", self._cb_detections, 10)
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)
        self.create_subscription(Vector3, "/arms/control_debug", self._cb_debug, 10)
        self.create_subscription(Image, "/arms/roi_image", self._cb_roi, best_effort_qos)
        # hsv_debug_image(≈500KB raw)는 cv_debug 패널일 때만 구독한다. 아니면
        # 구독자가 0이라 detection 이 발행 자체를 스킵 → UDP 부하/지연이 사라진다.
        if self._cv_debug:
            self.create_subscription(Image, "/arms/hsv_debug_image",
                                     self._cb_hsv_debug, best_effort_qos)
        # 실기체 command 퍼블리셔(arms_command_hw_node)가 BEST_EFFORT 라서 구독도 맞춰야
        # 메시지를 받는다. (RELIABLE 구독이면 BEST_EFFORT 발행을 하나도 못 받아 모드 전환
        # 오버레이/효과음이 동작하지 않음.)
        self.create_subscription(Joy, "/arms/command", self._cb_command, best_effort_qos)
        # arms_control 이 CRSF 배터리 텔레메트리를 표준 메시지로 재발행 (RELIABLE, depth 10).
        self.create_subscription(BatteryState, "/arms/battery", self._cb_battery, 10)
        # 검출기 상태 [yolo, hsv proposal, legacy slot] — SEARCH 표시용.
        self.create_subscription(Float32MultiArray, "/arms/detector_status",
                                 self._cb_detstatus, 10)

        self._latest_detections = DetectionArray()
        self._latest_state = MissionState()
        self._latest_cmd = Vector3()
        self._latest_roi = None
        self._latest_hsv_debug = None
        self._latest_battery = None   # /arms/battery 최신값 (None=아직 미수신)
        self._elrs_connected = None   # ELRS 연결 여부 (None=미수신, True/False). 엣지에 효과음
        # 시작 안내음: 모드+텔레메트리를 순서대로(모드 먼저) 한 번만, 공백 없이 이어 재생.
        #   이게 끝나기 전엔 개별 첫-수신 효과음을 내지 않아 순서가 보장된다.
        self._startup_sound_done = False
        self._det_status = None       # raw [yolo, hsv proposal, legacy slot]
        history = max(30, int(self.get_parameter("ui.diagnostics_history").value))
        self._diag_history = {
            name: deque(maxlen=history) for name in (
                "sample", "resize", "hsv", "yolo", "tracker", "total",
                "hsv_count", "yolo_inputs", "yolo_accepted")
        }
        # 검출기별 최근 유효(≥0) confidence 를 잠깐 유지해 깜빡임 억제: idx→(val, t)
        self._det_hold = {}
        self._manual_mode = False   # buttons[2]==1 → 수동 모드(오버레이 끔)
        self._prev_manual = None    # 이전 mode 값 (None=아직 미수신, 첫 수신은 효과음 없음)
        self._kill = False          # buttons[0]==1 → kill 스위치 ON (모드 무관 경고 표시)
        self._kill_blink = 0        # kill 경고 점멸용 프레임 카운터
        self._prev_kill = None      # kill 이전값 (off→on 상승 엣지에만 경고음, 첫 수신 skip)
        # mission_state 기반 상태 변경 효과음 엣지 추적
        self._prev_manual_state = None  # MissionState.manual_mode 이전값 (모드전환 틱 억제용)
        self._prev_armed = None         # MissionState.armed 이전값 (수동 arm/disarm)
        self._prev_state = None         # MissionState.state 이전값 (자동 idle/search)

        # 실기체 런치에서만 fullscreen:=true → 전체화면. SITL 등은 기본값 False(창 모드).
        self.declare_parameter("ui.fullscreen", False)
        self._fullscreen = self.get_parameter("ui.fullscreen").value
        self._keep_aspect = True    # True=레터박스(비율 유지, 잘림 없음), False=강제 스트레치

        # 터치(=마우스 클릭)로 누르는 전원 종료 버튼. 누르면 즉시 끄지 않고 확인 받는다.
        self.declare_parameter("ui.power_button", True)
        self.declare_parameter("ui.power_off_cmd", "systemctl poweroff")
        self._confirm_shutdown = False    # 확인 다이얼로그 표시 중 여부
        self._btn_rects = {}              # 버튼 히트박스(원본 프레임 좌표): name→(x1,y1,x2,y2)
        self._fit_xform = (0.0, 0.0, 1.0, 1.0)   # 표시좌표→원본좌표 역변환(x0,y0,sx,sy)

        cv2.namedWindow("A.R.M.S.", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("A.R.M.S.", self._on_mouse)
        if self._fullscreen:
            # 작업표시줄·타이틀바·상단바까지 가리는 진짜 전체화면
            cv2.setWindowProperty(
                "A.R.M.S.", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )
            self._screen_w, self._screen_h = self._detect_screen_size()
            self.get_logger().info(
                f"fullscreen mode, screen size = {self._screen_w}x{self._screen_h}")
        else:
            cv2.resizeWindow("A.R.M.S.", 960, 720)
            self._screen_w, self._screen_h = 0, 0
            self.get_logger().info("windowed mode (960x720)")

        # 상태에 따라 비프 간격을 조절하려고 50ms 마다 확인한다.
        self.create_timer(0.05, self._beep_tick)
        # 영상이 없어도 창이 뜨도록 대기화면을 주기적으로 그린다(카메라 미연결 대비).
        self.create_timer(0.1, self._ui_tick)

        self.get_logger().info("arms_ui_node started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_detections(self, msg: DetectionArray):
        self._latest_detections = msg

    def _cb_state(self, msg: MissionState):
        self._latest_state = msg

        # ---- 상태 변경 효과음 ----
        #   수동 모드: arm/disarm 엣지    자동 모드: idle↔search 엣지
        #   (모드 전환 자체의 효과음은 _cb_command 가 담당. 모드가 막 바뀐 틱에서는
        #    arm/state 값이 동시에 튀므로 중복 재생을 막기 위해 건너뛴다.)
        manual = bool(msg.manual_mode)
        armed = bool(msg.armed)
        state = msg.state or "IDLE"

        mode_changed = (self._prev_manual_state is not None and
                        manual != self._prev_manual_state)
        if not mode_changed:
            if manual:
                # 수동: arm/disarm
                if self._prev_armed is not None and armed != self._prev_armed:
                    param = "ui.sound_arm" if armed else "ui.sound_disarm"
                    self._play_sound(self.get_parameter(param).value)
            else:
                # 자동: idle ↔ search
                if self._prev_state is not None and state != self._prev_state:
                    if state == "SEARCH" and self._prev_state == "IDLE":
                        self._play_sound(self.get_parameter("ui.sound_search").value)
                    elif state == "IDLE" and self._prev_state == "SEARCH":
                        self._play_sound(self.get_parameter("ui.sound_idle").value)

        self._prev_manual_state = manual
        self._prev_armed = armed
        self._prev_state = state

        # ---- ELRS 연결/끊김 효과음 ----
        #   시작 안내는 _maybe_startup_announce(모드→텔레메트리 순). 이후 변화 엣지에만 개별.
        connected = bool(getattr(msg, "elrs_connected", False))
        if self._startup_sound_done and self._elrs_connected is not None \
                and connected != self._elrs_connected:
            param = "ui.sound_telm_connected" if connected else "ui.sound_telm_lost"
            self._play_sound(self.get_parameter(param).value)
        self._elrs_connected = connected
        self._maybe_startup_announce()

    def _maybe_startup_announce(self):
        """켜진 뒤 모드와 텔레메트리 연결여부가 둘 다 확인되면, 그때 한 번만
        '모드 → 텔레메트리' 순서로 공백 없이 이어 재생한다(순서 보장 + 사이 공백 제거)."""
        if self._startup_sound_done:
            return
        if self._prev_manual is None or self._elrs_connected is None:
            return   # 아직 둘 중 하나 미수신 → 대기
        mode_p = self.get_parameter(
            "ui.sound_manual" if self._manual_mode else "ui.sound_auto").value
        telm_p = self.get_parameter(
            "ui.sound_telm_connected" if self._elrs_connected
            else "ui.sound_telm_lost").value
        self._startup_sound_done = True
        self._play_sequence([mode_p, telm_p])   # 모드 먼저 → 텔레메트리, 공백 없이

    def _cb_debug(self, msg: Vector3):
        self._latest_cmd = msg

    def _cb_battery(self, msg: BatteryState):
        self._latest_battery = msg

    def _cb_detstatus(self, msg: Float32MultiArray):
        vals = list(msg.data)
        self._det_status = vals
        now = time.monotonic()
        for i, v in enumerate(vals):      # 유효(≥0) 값은 잠깐 유지해 깜빡임 방지
            if v >= 0.0:
                self._det_hold[i] = (v, now)
        if len(vals) >= 16:
            names = ("hsv_count", "yolo_inputs", "yolo_accepted", "sample",
                     "resize", "hsv", "yolo", "tracker", "total")
            for name, value in zip(names, vals[4:13]):
                self._diag_history[name].append(float(value))

    def _cb_command(self, msg: Joy):
        if len(msg.buttons) <= MODE_BUTTON_IDX:
            return
        kill = bool(msg.buttons[KILL_BUTTON_IDX])
        self._kill = kill
        # 킬 스위치 ON(off→on 상승 엣지)에만 경고음. 첫 수신(prev None)·끌 때는 재생 안 함.
        if self._prev_kill is not None and kill and not self._prev_kill:
            self._play_sound(self.get_parameter("ui.sound_kill").value)
        self._prev_kill = kill
        manual = bool(msg.buttons[MODE_BUTTON_IDX])
        self._manual_mode = manual
        # 시작 안내(모드+텔레메트리)는 _maybe_startup_announce 가 순서대로 한 번에 낸다.
        #   그 이후의 모드 전환 엣지에만 개별 효과음을 낸다.
        if self._startup_sound_done and self._prev_manual is not None \
                and manual != self._prev_manual:
            param = "ui.sound_manual" if manual else "ui.sound_auto"
            self._play_sound(self.get_parameter(param).value)
        self._prev_manual = manual
        self._maybe_startup_announce()

    def _play_sound(self, path):
        """ffplay 로 효과음을 비동기 즉시 재생 (UI 렌더링을 막지 않음, 지연 없음)."""
        # 트리거가 실제로 걸렸는지 항상 로그로 남긴다 (재생 성공 여부와 별개).
        #   → 로그는 뜨는데 소리가 없으면 오디오/ffplay 문제, 로그도 없으면 토픽/필드 문제.
        self.get_logger().info(
            f"효과음 트리거: {os.path.basename(path) if path else path}")
        if self._player is None:
            self.get_logger().warn("ffplay 없음 — 효과음 재생 불가")
            return
        if not path or not os.path.isfile(path):
            self.get_logger().warn(f"효과음 파일 없음: {path}")
            return

        def _run():
            try:
                subprocess.run(
                    [self._player, "-nodisp", "-autoexit", "-loglevel", "quiet", path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as e:
                self.get_logger().warn(f"효과음 재생 실패: {e}")

        threading.Thread(target=_run, daemon=True).start()

    def _play_sequence(self, paths):
        """여러 효과음을 하나의 ffplay(concat 프로토콜)로 순서대로·공백 없이 이어 재생.
        시작 안내(모드→텔레메트리)처럼 순서가 중요한 경우에만 쓴다. 비동기(즉시)."""
        paths = [p for p in paths if p and os.path.isfile(p)]
        if not paths or self._player is None:
            return
        src = paths[0] if len(paths) == 1 else "concat:" + "|".join(paths)

        def _run():
            try:
                subprocess.run(
                    [self._player, "-nodisp", "-autoexit", "-loglevel", "quiet", "-i", src],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as e:
                self.get_logger().warn(f"효과음 재생 실패: {e}")

        threading.Thread(target=_run, daemon=True).start()

    # ------------------------------------------------------------------
    # 탐지 비프음 (상태별 간격)
    # ------------------------------------------------------------------

    def _ui_tick(self):
        """영상 콜백이 최근에 안 돌았으면(카메라 미연결 등) 대기화면을 직접 표시해
        UI 창이 항상 뜨게 한다. 전원 버튼·배터리도 그려 조작 가능하게 유지."""
        if time.monotonic() - self._last_image_t < 0.7:
            return   # 영상이 흐르는 중 → _cb_image 가 표시 담당
        if self._fullscreen and self._screen_w > 0 and self._screen_h > 0:
            h, w = self._screen_h, self._screen_w
        else:
            h, w = 720, 960
        frame = np.full((h, w, 3), 30, dtype=np.uint8)
        self._put_center_text(frame, "NO CAMERA SIGNAL", int(h * 0.5),
                              (0, 0, 255), max(0.8, w / 900.0), 2)
        # 대기화면에선 중앙 십자선·부제를 그리지 않는다(요청).
        self._draw_link_status(frame)            # 모드·TELM 연결 표시
        self._draw_battery(frame)
        self._draw_power_ui(frame)               # 전원 버튼 사용 가능하게
        self._fit_xform = (0.0, 0.0, 1.0, 1.0)   # 이미 화면 크기 → 항등(터치 매핑)
        cv2.imshow("A.R.M.S.", frame)
        cv2.waitKey(1)

    def _beep_tick(self):
        """SEARCH=느린 간헐 비프, LOCK 이상=끊기지 않는 연속 톤. AUTO 모드 한정."""
        if (not self.get_parameter("ui.beep_enabled").value) or self._manual_mode:
            self._stop_lock_tone()   # 비활성/수동 모드에선 연속음도 끈다.
            return
        state = self._latest_state.state or "IDLE"
        if state == "SEARCH":
            self._stop_lock_tone()
            interval = float(self.get_parameter("ui.beep_search_interval").value)
            now = time.monotonic()
            if now - self._last_beep_t >= interval:
                self._last_beep_t = now
                self._play_beep()
        elif state in ("LOCK", "BOOST", "TRACK", "FIRE"):
            self._start_lock_tone()  # LOCK 걸리면 삐이이——— 계속 울린다.
        else:
            self._stop_lock_tone()   # IDLE / RTL 등은 무음

    def _start_lock_tone(self):
        """LOCK 이상 상태: 연속 사인 톤을 재생 (이미 재생 중이면 그대로 둔다)."""
        if self._player is None:
            return
        if self._lock_tone_proc is not None and self._lock_tone_proc.poll() is None:
            return                   # 이미 울리는 중
        freq = float(self.get_parameter("ui.beep_freq_hz").value)
        vol = float(self.get_parameter("ui.beep_volume").value)
        try:
            # duration 없는 사인 = 무한 톤 → terminate 할 때까지 계속 울린다.
            self._lock_tone_proc = subprocess.Popen(
                [self._player, "-nodisp", "-autoexit", "-loglevel", "quiet",
                 "-f", "lavfi", "-i", f"sine=frequency={freq}", "-af", f"volume={vol}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warn(f"LOCK 연속음 재생 실패: {e}")
            self._lock_tone_proc = None

    def _stop_lock_tone(self):
        """연속 톤을 중지한다 (재생 중이 아니면 무시)."""
        p = self._lock_tone_proc
        if p is None:
            return
        self._lock_tone_proc = None
        try:
            if p.poll() is None:
                p.terminate()
        except Exception:
            pass

    def _play_beep(self):
        """ffplay lavfi 사인파로 짧은 비프음을 비동기 재생 (mp3 파일 불필요)."""
        if self._player is None:
            return
        freq = float(self.get_parameter("ui.beep_freq_hz").value)
        dur = float(self.get_parameter("ui.beep_duration").value)
        vol = float(self.get_parameter("ui.beep_volume").value)

        def _run():
            try:
                subprocess.run(
                    [self._player, "-nodisp", "-autoexit", "-loglevel", "quiet",
                     "-f", "lavfi", "-i", f"sine=frequency={freq}:duration={dur}",
                     "-af", f"volume={vol}"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as e:
                self.get_logger().warn(f"비프음 재생 실패: {e}")

        threading.Thread(target=_run, daemon=True).start()

    def _cb_roi(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
            return
        try:
            self._latest_roi = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"roi cv_bridge error: {e}")

    def _cb_hsv_debug(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
            return
        try:
            self._latest_hsv_debug = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"hsv debug cv_bridge error: {e}")

    def _cb_image(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
            return   # 빈 프레임(image_publisher 루프 경계 등) → 스킵
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return
        if frame is None or frame.size == 0:
            return
        self._last_image_t = time.monotonic()   # 대기화면 타이머가 라이브로 양보하게

        # 수동 모드에서는 원본 영상 + arm/disarm 표시, 오버레이는 생략한다.
        if not self._manual_mode:
            self._draw_overlay(frame)
            # IDLE 에선 표적 확대 뷰(ROI PiP)도 표시하지 않는다.
            if (self._latest_state.state or "IDLE") != "IDLE":
                self._draw_roi_pip(frame)
        else:
            self._draw_manual_arm(frame)
            # arm 스위치 올렸는데 스틱이 idle 아니면 arm 차단 경고.
            self._draw_prearm_banner(frame)
        # 중앙 조준 십자선은 상태·모드 무관하게 항상 표시 (IDLE 포함).
        self._draw_crosshair(frame)
        # 현재 모드(AUTO/MANUAL)·ELRS 연결 여부는 상단 중앙에 항상 표시.
        self._draw_link_status(frame)
        # 배터리(전압/퍼센트)는 하단 중앙에 자동/수동 무관하게 항상 표시.
        self._draw_battery(frame)
        # 검출기 작동여부·confidence·모드 텍스트는 디버그 모드에서만(메인은 깔끔하게).
        if self._debug and not self._manual_mode and (
                self._latest_state.state or "IDLE") in (
                "SEARCH", "LOCK", "BOOST", "TRACK", "FIRE"):
            self._draw_detector_status(frame)
        # kill 스위치 ON 이면 자동/수동 무관하게 화면 중앙에 크게 경고 (최상단).
        self._draw_kill_banner(frame)
        # 전원 버튼(+확인 다이얼로그)은 최상단에 그린다(원본 좌표계, 터치 히트박스 기록).
        self._draw_power_ui(frame)
        if self._cv_debug:
            frame = self._compose_diagnostics_panel(frame)
        if self._fullscreen:
            frame = self._fit_to_window(frame)
        else:
            self._fit_xform = (0.0, 0.0, 1.0, 1.0)   # 창 모드: 표시=원본 좌표(항등)
        cv2.imshow("A.R.M.S.", frame)
        cv2.waitKey(1)

    def _compose_diagnostics_panel(self, frame):
        """영상 오른쪽에 HSV 후보 미리보기와 실시간 성능 그래프를 붙인다."""
        h, w = frame.shape[:2]
        panel_w = max(320, int(w * 0.40))
        panel = np.full((h, panel_w, 3), 22, dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(panel, "DETECTION DIAGNOSTICS", (12, 25), font, 0.58,
                    (240, 240, 240), 1, cv2.LINE_AA)

        raw = self._det_status or []
        frame_no = int(raw[14]) if len(raw) >= 15 else 0
        tracker_names = {0: "ACQUIRE", 1: "TRACK", 2: "LOST"}
        tracker = tracker_names.get(int(raw[13]), "--") if len(raw) >= 14 else "--"
        mode = ("FULL-FALLBACK" if len(raw) >= 16 and raw[15] >= 0.5 else
                ("ROI" if len(raw) >= 4 and raw[3] >= 0.5 else "HSV->YOLO"))
        cv2.putText(panel, f"frame {frame_no}  {tracker}  {mode}", (12, 47),
                    font, 0.46, (0, 210, 255), 1, cv2.LINE_AA)

        # 같은 실시간 스트림에서 만들어진 HSV probability overlay + 후보 박스.
        preview_y = 58
        preview_h = max(105, int(h * 0.24))
        cv2.rectangle(panel, (10, preview_y),
                      (panel_w - 10, preview_y + preview_h), (65, 65, 65), 1)
        preview = self._latest_hsv_debug
        if preview is not None and preview.size:
            ph, pw = preview.shape[:2]
            scale = min((panel_w - 22) / pw, (preview_h - 2) / ph)
            nw, nh = max(1, int(pw * scale)), max(1, int(ph * scale))
            small = cv2.resize(preview, (nw, nh), interpolation=cv2.INTER_AREA)
            x0 = (panel_w - nw) // 2
            y0 = preview_y + (preview_h - nh) // 2
            panel[y0:y0 + nh, x0:x0 + nw] = small
        else:
            cv2.putText(panel, "waiting for HSV candidates", (20, preview_y + preview_h // 2),
                        font, 0.44, (140, 140, 140), 1, cv2.LINE_AA)

        timing_y = preview_y + preview_h + 20
        graph_h = max(105, int(h * 0.25))
        timing_series = [
            ("sample", (180, 180, 180)), ("total", (0, 220, 255)),
            ("yolo", (0, 90, 255)), ("hsv", (255, 180, 0)),
            ("tracker", (100, 220, 100)),
        ]
        self._draw_history_graph(panel, (10, timing_y, panel_w - 10,
                                         min(h - 130, timing_y + graph_h)),
                                 "TIME (ms)", timing_series, minimum_scale=33.3)

        count_y = min(h - 120, timing_y + graph_h + 18)
        self._draw_history_graph(
            panel, (10, count_y, panel_w - 10, h - 54), "COUNTS",
            [("hsv_count", (255, 180, 0)),
             ("yolo_inputs", (0, 130, 255)),
             ("yolo_accepted", (0, 220, 0))], minimum_scale=2.0,
            integer_scale=True)

        def latest(name):
            values = self._diag_history[name]
            return values[-1] if values else 0.0

        footer = (f"HSV {latest('hsv_count'):.0f}  YOLO in/ok "
                  f"{latest('yolo_inputs'):.0f}/{latest('yolo_accepted'):.0f}   "
                  f"total {latest('total'):.1f} ms")
        cv2.putText(panel, footer, (10, h - 18), font, 0.42,
                    (230, 230, 230), 1, cv2.LINE_AA)
        cv2.line(panel, (0, 0), (0, h - 1), (90, 90, 90), 2)
        return np.hstack((frame, panel))

    def _draw_history_graph(self, canvas, rect, title, series,
                            minimum_scale=1.0, integer_scale=False):
        """외부 plotting 라이브러리 없이 가벼운 rolling line graph를 그린다."""
        x1, y1, x2, y2 = rect
        if x2 - x1 < 40 or y2 - y1 < 45:
            return
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (48, 48, 48), -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (85, 85, 85), 1)
        values = [v for name, _ in series for v in self._diag_history[name]]
        scale_max = max(minimum_scale, float(np.percentile(values, 95)) * 1.25
                        if values else minimum_scale)
        if integer_scale:
            scale_max = max(minimum_scale, float(np.ceil(scale_max)))
        cv2.putText(canvas, f"{title}  max {scale_max:.0f}", (x1 + 5, y1 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1,
                    cv2.LINE_AA)
        plot_y1, plot_y2 = y1 + 22, y2 - 18
        for frac in (0.0, 0.5, 1.0):
            yy = int(plot_y2 - frac * (plot_y2 - plot_y1))
            cv2.line(canvas, (x1 + 4, yy), (x2 - 4, yy), (58, 58, 58), 1)
        legend_x = x1 + 5
        for name, color in series:
            data = list(self._diag_history[name])
            if len(data) >= 2:
                xs = np.linspace(x1 + 4, x2 - 4, len(data)).astype(np.int32)
                ys = np.array([
                    int(plot_y2 - min(scale_max, max(0.0, value)) /
                        scale_max * (plot_y2 - plot_y1)) for value in data
                ], dtype=np.int32)
                points = np.column_stack((xs, ys)).reshape((-1, 1, 2))
                cv2.polylines(canvas, [points], False, color, 1, cv2.LINE_AA)
            cv2.putText(canvas, name, (legend_x, y2 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, color, 1, cv2.LINE_AA)
            legend_x += max(42, 7 * len(name))

    def _detect_screen_size(self):
        """실제 모니터 해상도 감지. xrandr 우선, 실패 시 창 rect/기본값."""
        try:
            out = subprocess.check_output(
                ["xrandr"], stderr=subprocess.DEVNULL).decode()
            for line in out.splitlines():
                if "*" in line:                    # 현재 활성 모드 (예: "1920x1080 60.00*+")
                    res = line.split()[0]
                    w, h = res.split("x")
                    return int(w), int(h)
        except Exception:
            pass
        try:
            _, _, win_w, win_h = cv2.getWindowImageRect("A.R.M.S.")
            if win_w > 0 and win_h > 0:
                return win_w, win_h
        except Exception:
            pass
        return 1920, 1080                          # 최종 fallback

    def _fit_to_window(self, frame):
        """모니터 해상도에 맞춰 프레임을 확대해 화면을 채운다.

        _keep_aspect=True  → 비율 유지(레터박스, 좌우 검은 여백)
        _keep_aspect=False → 강제 스트레치(여백 없이 꽉 채움, 비율 깨질 수 있음)
        """
        win_w, win_h = self._screen_w, self._screen_h
        if win_w <= 0 or win_h <= 0:
            self._fit_xform = (0.0, 0.0, 1.0, 1.0)
            return frame
        h, w = frame.shape[:2]
        if not self._keep_aspect:
            # 강제 스트레치: x/y 배율이 다름, 여백 없음.
            self._fit_xform = (0.0, 0.0, win_w / w, win_h / h)
            return cv2.resize(frame, (win_w, win_h),
                              interpolation=cv2.INTER_LINEAR)
        scale = min(win_w / w, win_h / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((win_h, win_w, 3), dtype=np.uint8)
        x0 = (win_w - new_w) // 2
        y0 = (win_h - new_h) // 2
        canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
        # 표시(canvas)좌표 → 원본프레임 좌표 역변환 파라미터 저장(터치 매핑용).
        self._fit_xform = (float(x0), float(y0), float(scale), float(scale))
        return canvas

    def _draw_roi_pip(self, frame):
        """추적 ROI 확대 뷰를 우측 하단 PiP 로 합성."""
        roi = self._latest_roi
        if roi is None or roi.size == 0:
            return
        h, w = frame.shape[:2]
        frac = float(self.get_parameter("ui.roi_pip_frac").value)
        pip_w = max(1, int(w * frac))
        pip_h = max(1, int(pip_w * roi.shape[0] / roi.shape[1]))
        pip_h = min(pip_h, h // 2)   # 너무 세로로 길어지지 않게 제한
        pip = cv2.resize(roi, (pip_w, pip_h), interpolation=cv2.INTER_NEAREST)
        m = 8  # 가장자리 여백
        x2, y2 = w - m, h - m
        x1, y1 = x2 - pip_w, y2 - pip_h
        if x1 < 0 or y1 < 0:
            return
        frame[y1:y2, x1:x2] = pip
        cv2.rectangle(frame, (x1 - 1, y1 - 1), (x2, y2), (0, 220, 0), 2)
        cv2.putText(frame, "ROI", (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _put_center_text(self, frame, text, cy, color, scale, thickness):
        """가로 중앙 정렬 텍스트 + 검은 외곽선(영상 위 가독성 확보)."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        x = (frame.shape[1] - tw) // 2
        y = cy + th // 2
        cv2.putText(frame, text, (x, y), font, scale, (0, 0, 0),
                    thickness + 3, cv2.LINE_AA)
        cv2.putText(frame, text, (x, y), font, scale, color,
                    thickness, cv2.LINE_AA)

    def _draw_manual_arm(self, frame):
        """수동 모드: arm/disarm 여부를 상단 중앙에 표시."""
        h = frame.shape[0]
        armed = bool(self._latest_state.armed)
        text = "ARMED" if armed else "DISARMED"
        color = (0, 0, 255) if armed else (0, 200, 0)   # arm=빨강(주의), disarm=초록(안전)
        scale = max(0.8, h / 600.0)
        self._put_center_text(frame, text, int(h * 0.08), color, scale, 2)

    def _draw_prearm_banner(self, frame):
        """수동: arm 스위치 올렸지만 스틱이 idle 아니어서 arm 차단됨 → 주황 경고 배너."""
        if not bool(getattr(self._latest_state, "prearm_blocked", False)):
            return
        h, w = frame.shape[:2]
        band_h = int(h * 0.15)
        y0 = int(h * 0.60)
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, y0), (w, y0 + band_h), (0, 110, 200), -1)  # 주황(BGR)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        text = "ARM DENIED - STICKS NOT IDLE"
        base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)[0][0]
        scale = (w * 0.7) / max(1, base)
        self._put_center_text(frame, text, y0 + band_h // 2, (255, 255, 255), scale, 3)

    def _draw_kill_banner(self, frame):
        """kill 스위치 ON: 모드 무관하게 화면 중앙에 크게 점멸 경고."""
        if not self._kill:
            return
        h, w = frame.shape[:2]
        self._kill_blink = (self._kill_blink + 1) % 20
        # 붉은 테두리는 항상, 중앙 배너/텍스트는 점멸(대략 65% 켜짐)
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), (0, 0, 255), 12)
        if self._kill_blink >= 13:
            return
        band_h = int(h * 0.24)
        y0 = (h - band_h) // 2
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, y0), (w, y0 + band_h), (0, 0, 150), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        # 텍스트 폭이 화면의 ~80% 가 되도록 스케일 자동 계산
        text = "KILL SWITCH ENGAGED"
        base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 4)[0][0]
        scale = (w * 0.8) / max(1, base)
        self._put_center_text(frame, text, h // 2, (255, 255, 255), scale, 4)

    # ------------------------------------------------------------------
    # 전원 종료 버튼 (터치/클릭) + 확인 다이얼로그
    # ------------------------------------------------------------------

    @staticmethod
    def _put_text_in_rect(frame, text, x1, y1, x2, y2, color, scale, thick):
        """사각형 중앙에 외곽선+본문 텍스트."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        tx = x1 + ((x2 - x1) - tw) // 2
        ty = y1 + ((y2 - y1) + th) // 2
        cv2.putText(frame, text, (tx, ty), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(frame, text, (tx, ty), font, scale, color, thick, cv2.LINE_AA)

    def _draw_power_ui(self, frame):
        """우상단 전원(OFF) 버튼과, 눌렀을 때의 종료 확인 다이얼로그를 그린다.
        터치 히트박스는 self._btn_rects(원본 프레임 좌표)에 기록한다."""
        self._btn_rects = {}
        if not self.get_parameter("ui.power_button").value:
            self._confirm_shutdown = False
            return
        h, w = frame.shape[:2]
        m  = max(8, int(h * 0.02))
        bs = max(46, int(h * 0.12))
        x2, y1 = w - m, m
        x1, y2 = x2 - bs, y1 + bs
        self._btn_rects["power"] = (x1, y1, x2, y2)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (30, 30, 200), -1)   # 빨강(BGR)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        self._put_text_in_rect(frame, "OFF", x1, y1, x2, y2,
                               (255, 255, 255), max(0.5, bs / 90.0), 2)

        if self._confirm_shutdown:
            self._draw_confirm_dialog(frame)

    def _draw_confirm_dialog(self, frame):
        """화면 중앙에 '정말 끄겠습니까?' 확인 모달 (YES/NO 버튼)."""
        h, w = frame.shape[:2]
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)      # 화면 어둡게
        bw, bh = int(w * 0.6), int(h * 0.42)
        bx1, by1 = (w - bw) // 2, (h - bh) // 2
        bx2, by2 = bx1 + bw, by1 + bh
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (45, 45, 45), -1)
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (255, 255, 255), 2)
        self._put_text_in_rect(frame, "SHUT DOWN?", bx1, by1, bx2,
                               by1 + int(bh * 0.42), (255, 255, 255),
                               max(0.7, w / 900.0), 2)
        # YES / NO 버튼
        btn_w, btn_h = int(bw * 0.38), int(bh * 0.30)
        gap = int(bw * 0.08)
        bxs = bx1 + (bw - (btn_w * 2 + gap)) // 2
        byb = by2 - btn_h - int(bh * 0.12)
        yx1, yx2 = bxs, bxs + btn_w
        nx1, nx2 = yx2 + gap, yx2 + gap + btn_w
        cv2.rectangle(frame, (yx1, byb), (yx2, byb + btn_h), (30, 30, 200), -1)  # YES 빨강
        cv2.rectangle(frame, (nx1, byb), (nx2, byb + btn_h), (90, 110, 90), -1)  # NO 회색초록
        self._btn_rects["yes"] = (yx1, byb, yx2, byb + btn_h)
        self._btn_rects["no"]  = (nx1, byb, nx2, byb + btn_h)
        bscale = max(0.6, btn_h / 70.0)
        self._put_text_in_rect(frame, "YES", yx1, byb, yx2, byb + btn_h, (255, 255, 255), bscale, 2)
        self._put_text_in_rect(frame, "NO",  nx1, byb, nx2, byb + btn_h, (255, 255, 255), bscale, 2)

    def _on_mouse(self, event, x, y, flags, param):
        """터치=좌클릭. 표시좌표를 원본좌표로 역변환해 버튼 히트박스와 비교."""
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        x0, y0, sx, sy = self._fit_xform
        rx = (x - x0) / sx if sx else x
        ry = (y - y0) / sy if sy else y

        def hit(name):
            r = self._btn_rects.get(name)
            return r is not None and r[0] <= rx <= r[2] and r[1] <= ry <= r[3]

        if self._confirm_shutdown:
            if hit("yes"):
                self._confirm_shutdown = False
                self._do_power_off()
            elif hit("no"):
                self._confirm_shutdown = False       # 취소
        elif hit("power"):
            self._confirm_shutdown = True             # 확인 다이얼로그 띄움(바로 안 끔)

    def _do_power_off(self):
        """확인된 경우에만 전원 종료 명령 실행."""
        cmd = str(self.get_parameter("ui.power_off_cmd").value)
        self.get_logger().warn(f"전원 종료 확인됨 → 실행: {cmd}")
        try:
            subprocess.Popen(cmd, shell=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f"전원 종료 명령 실패: {e}")

    def _draw_crosshair(self, frame):
        """화면 정중앙 조준 십자선 — 자동/수동 모두 항상 표시."""
        h, w = frame.shape[:2]
        cx_f, cy_f = w // 2, h // 2
        cv2.line(frame, (cx_f - 20, cy_f), (cx_f + 20, cy_f), (0, 255, 0), 1)
        cv2.line(frame, (cx_f, cy_f - 20), (cx_f, cy_f + 20), (0, 255, 0), 1)
        cv2.circle(frame, (cx_f, cy_f), 2, (0, 255, 0), -1)

    def _draw_link_status(self, frame):
        """현재 모드(TRACKING/MANUAL)와 TELM 연결 여부를 상단 중앙에 한 줄로 표시 — 항상.
        TELM 슬롯 폭을 가장 긴 'TELM LOST' 로 고정해, 연결상태가 바뀌어도 MODE 위치가 안 밀린다."""
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.5, h / 900.0)
        thick = max(1, int(round(scale * 2)))
        mode_txt = "MANUAL" if self._manual_mode else "TRACKING"
        if self._elrs_connected is None:
            link_txt, link_col = "", (0, 0, 0)   # 아직 미수신 → TELM 표시 안 함(모드만)
        elif self._elrs_connected:
            link_txt, link_col = "TELM OK", (0, 220, 0)
        else:
            # 끊김은 안전상 눈에 띄게 점멸(빨강).
            self._elrs_blink = (getattr(self, "_elrs_blink", 0) + 1) % 20
            link_col = (60, 60, 60) if self._elrs_blink >= 13 else (0, 0, 255)
            link_txt = "TELM LOST"
        sep = "   "
        mode_str = mode_txt + sep
        (mw, th), _ = cv2.getTextSize(mode_str, font, scale, thick)
        (slot_w, _), _ = cv2.getTextSize("TELM LOST", font, scale, thick)   # 슬롯 폭 고정
        x = (w - (mw + slot_w)) // 2
        y = max(th + 6, int(h * 0.035))
        # 외곽선 → 가독성. 모드=흰색, TELM=상태색. (각각 따로 그려 슬롯 고정 유지)
        cv2.putText(frame, mode_str, (x, y), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(frame, mode_str, (x, y), font, scale, (255, 255, 255), thick, cv2.LINE_AA)
        cv2.putText(frame, link_txt, (x + mw, y), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(frame, link_txt, (x + mw, y), font, scale, link_col, thick, cv2.LINE_AA)

    def _draw_battery(self, frame):
        """배터리 전압/잔량을 하단 중앙에 표시 — 자동/수동·디버그 무관하게 항상
        (데이터 있을 때만; /arms/battery 미수신이면 표시할 값이 없어 생략)."""
        bat = self._latest_battery
        if bat is None:
            return
        h, w = frame.shape[:2]
        volt = float(bat.voltage)
        pct = float(bat.percentage) * 100.0   # BatteryState.percentage 는 0~1 규약
        pct_valid = (pct == pct) and pct >= 0   # NaN(pct!=pct)·음수면 무효
        if pct_valid:
            pct = max(0.0, min(100.0, pct))
            text = f"{volt:.1f}V  {pct:.0f}%"
        else:
            text = f"{volt:.1f}V"

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.5, h / 900.0)
        thick = max(1, int(round(scale * 2)))
        (tw, th), bl = cv2.getTextSize(text, font, scale, thick)
        m = max(10, int(h * 0.02))
        x = (w - tw) // 2               # 가로 중앙
        y = h - m                       # 하단 (텍스트 baseline)
        # 배경 박스 없이 검은 외곽선 + 흰 글씨만 (영상 위 최소 가독성).
        cv2.putText(frame, text, (x, y), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(frame, text, (x, y), font, scale, (255, 255, 255), thick, cv2.LINE_AA)

    def _draw_detector_status(self, frame):
        """YOLO/HSV proposal 상태와 검출 모드(FULL/ROI)를 좌상단에 표시."""
        raw = self._det_status
        if raw is None or len(raw) < 3:
            return
        h = frame.shape[0]
        now = time.monotonic()
        names = ("YOLO", "HSV proposal")
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.45, h / 1100.0)
        thick = max(1, int(round(scale * 2)))
        x = 10
        # _draw_overlay 의 err(10,60)·cmd(10,80) 숫자 디버그 텍스트 아래에서 시작해
        # LOCK/TRACK/FIRE 에서 서로 겹치지 않게 한다.
        y = int(100 * max(1.0, h / 480.0))
        dy = int(cv2.getTextSize("A", font, scale, thick)[0][1] * 2.2)

        # 검출 모드 헤더: FULL=전체프레임(획득), ROI=크롭 안에서만(추적) → '가중치' 전환 확인용
        mode = raw[3] if len(raw) >= 4 else 0.0
        mtext = "MODE: ROI (crop)" if mode >= 0.5 else "MODE: FULL frame"
        mcolor = (0, 200, 255) if mode >= 0.5 else (200, 200, 200)
        cv2.putText(frame, mtext, (x, y), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(frame, mtext, (x, y), font, scale, mcolor, thick, cv2.LINE_AA)
        y += dy

        for i, name in enumerate(names):
            rv = raw[i]
            held = self._det_hold.get(i)
            fresh = held is not None and (now - held[1]) < 0.7    # 최근 0.7s 내 유효값
            if rv == -2.0:                      # 사용불가 (YOLO OOM 등)
                text, color = f"{name}: DEAD", (0, 0, 255)
            elif fresh:                          # 최근 confidence 있음
                pct = held[0] * 100.0
                color = (0, 220, 0) if held[0] > 0.0 else (170, 170, 170)
                text = f"{name}: {pct:2.0f}%" if held[0] > 0.0 else f"{name}: --"
            elif rv == -1.0:                     # 꺼짐/이 프레임 미실행
                text, color = f"{name}: off", (140, 140, 140)
            else:                                # 실행했으나 검출 없음
                text, color = f"{name}: --", (170, 170, 170)
            yy = y + i * dy
            cv2.putText(frame, text, (x, yy), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
            cv2.putText(frame, text, (x, yy), font, scale, color, thick, cv2.LINE_AA)

    def _draw_overlay(self, frame):
        h, w = frame.shape[:2]
        cx_f, cy_f = w // 2, h // 2   # 오차/명령 화살표 기준(중앙). 십자선 분리 후에도 필요.
        state = self._latest_state.state or "IDLE"
        color = STATE_COLORS.get(state, (255, 255, 255))

        # --- Bounding boxes (IDLE 에선 표적 표시 안 함) ---
        if state != "IDLE":
            for det in self._latest_detections.detections:
                cx = int(det.x_center * w)
                cy = int(det.y_center * h)
                bw = int(det.width * w)
                bh = int(det.height * h)
                x1, y1 = cx - bw // 2, cy - bh // 2
                x2, y2 = cx + bw // 2, cy + bh // 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{det.confidence:.2f}", (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # --- Crosshair 는 _draw_crosshair 로 분리(항상 표시) → 여기선 생략 ---

        # --- State label ---
        cv2.putText(frame, state, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        # --- Lock progress bar (SEARCH / LOCK) ---
        if state in ("SEARCH", "LOCK"):
            # control 이 발행한 실제 lock_duration_sec 를 기준으로 채운다(하드코딩 금지).
            # 예전엔 2.0 하드코딩이라, 실제 0.3s 만에 LOCK 됐는데 바는 15%만 차 보였다.
            lock_duration = float(getattr(self._latest_state, "lock_duration_sec", 0.0))
            if lock_duration <= 0.0:
                lock_duration = 2.0   # 아직 미수신/구버전 메시지 fallback
            progress = min(1.0, self._latest_state.lock_elapsed_sec / lock_duration)
            bar_w = int(w * 0.2)
            bar_x, bar_y = 10, h - 20
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 10), (80, 80, 80), -1)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(bar_w * progress), bar_y + 10), color, -1)
            cv2.putText(frame, "LOCK", (bar_x, bar_y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # --- Error/command 화살표 + 숫자 텍스트: 디버그 모드에서만 (bbox 는 위에서 항상) ---
        if self._debug and state in ("LOCK", "TRACK", "FIRE"):
            ex = int(self._latest_state.error_x * w)
            ey = int(self._latest_state.error_y * h)
            cv2.arrowedLine(frame, (cx_f, cy_f), (cx_f + ex, cy_f + ey),
                            (0, 255, 255), 2, tipLength=0.2)
            cv2.putText(frame, "ERR(target)", (cx_f + ex + 5, cy_f + ey),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            # --- 실제 나가는 제어 명령 : 빨강 = roll(가로)/pitch(세로) 각속도 ---
            #   control_debug 의 x,y 는 각도가 아니라 각속도[deg/s] 다 (ACRO 고정,
            #   각도 단계 삭제). 화살표 길이가 3.5배 튀지 않도록 배율도 같이 나눴다.
            roll = self._latest_cmd.x
            pitch = self._latest_cmd.y
            thr = self._latest_cmd.z
            scale = 8 / 3.5  # deg/s 당 픽셀 (옛 8 px/deg 와 같은 화면 길이)
            rx = int(roll * scale)
            ry = int(pitch * scale)
            cv2.arrowedLine(frame, (cx_f, cy_f), (cx_f + rx, cy_f + ry),
                            (0, 0, 255), 2, tipLength=0.2)
            cv2.putText(frame, "CMD(rate deg/s)", (cx_f + rx + 5, cy_f + ry + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # --- 숫자 디버그 텍스트 (좌상단) ---
            cv2.putText(frame, f"err  x={self._latest_state.error_x:+.2f} y={self._latest_state.error_y:+.2f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(frame, f"cmd  r={roll:+.0f} p={pitch:+.0f} deg/s  thr={thr:.2f}",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # --- State border ---
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), color, 3)

def main(args=None):
    rclpy.init(args=args)
    node = ArmsUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_lock_tone()   # 종료 시 연속음 프로세스 정리
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
