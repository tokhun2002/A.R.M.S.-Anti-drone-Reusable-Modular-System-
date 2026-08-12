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

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Joy

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
        self._player = shutil.which("ffplay")
        if self._player is None:
            self.get_logger().warn("ffplay 없음 — 효과음이 재생되지 않습니다.")

        self.create_subscription(Image, "/arms/image_raw", self._cb_image, best_effort_qos)
        self.create_subscription(DetectionArray, "/arms/detections", self._cb_detections, 10)
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)
        self.create_subscription(Vector3, "/arms/control_debug", self._cb_debug, 10)
        self.create_subscription(Image, "/arms/roi_image", self._cb_roi, best_effort_qos)
        # 실기체 command 퍼블리셔(arms_command_hw_node)가 BEST_EFFORT 라서 구독도 맞춰야
        # 메시지를 받는다. (RELIABLE 구독이면 BEST_EFFORT 발행을 하나도 못 받아 모드 전환
        # 오버레이/효과음이 동작하지 않음.)
        self.create_subscription(Joy, "/arms/command", self._cb_command, best_effort_qos)

        self._latest_detections = DetectionArray()
        self._latest_state = MissionState()
        self._latest_cmd = Vector3()
        self._latest_roi = None
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

        cv2.namedWindow("A.R.M.S.", cv2.WINDOW_NORMAL)
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

    def _cb_debug(self, msg: Vector3):
        self._latest_cmd = msg

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
        # 첫 수신(현재 모드 안내) 또는 전환 엣지에서 효과음.
        #   → 전체 시스템 런치 시 지금 어느 모드인지 소리로 알려준다.
        if self._prev_manual is None or manual != self._prev_manual:
            param = "ui.sound_manual" if manual else "ui.sound_auto"
            self._play_sound(self.get_parameter(param).value)
        self._prev_manual = manual

    def _play_sound(self, path):
        """ffplay 로 효과음을 비동기 재생 (UI 렌더링을 막지 않음)."""
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

    def _cb_roi(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
            return
        try:
            self._latest_roi = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"roi cv_bridge error: {e}")

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
        # kill 스위치 ON 이면 자동/수동 무관하게 화면 중앙에 크게 경고 (최상단).
        self._draw_kill_banner(frame)
        if self._fullscreen:
            frame = self._fit_to_window(frame)
        cv2.imshow("A.R.M.S.", frame)
        cv2.waitKey(1)

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
            return frame
        if not self._keep_aspect:
            return cv2.resize(frame, (win_w, win_h),
                              interpolation=cv2.INTER_LINEAR)
        h, w = frame.shape[:2]
        scale = min(win_w / w, win_h / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((win_h, win_w, 3), dtype=np.uint8)
        x0 = (win_w - new_w) // 2
        y0 = (win_h - new_h) // 2
        canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
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

    def _draw_crosshair(self, frame):
        """화면 정중앙 조준 십자선 — 자동/수동 모두 항상 표시."""
        h, w = frame.shape[:2]
        cx_f, cy_f = w // 2, h // 2
        cv2.line(frame, (cx_f - 20, cy_f), (cx_f + 20, cy_f), (0, 255, 0), 1)
        cv2.line(frame, (cx_f, cy_f - 20), (cx_f, cy_f + 20), (0, 255, 0), 1)
        cv2.circle(frame, (cx_f, cy_f), 2, (0, 255, 0), -1)

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

        # --- 현재 P (TRACK/FIRE 때 시간 램프 값 표시) ---
        if state in ("TRACK", "FIRE"):
            cv2.putText(frame, f"P {self._latest_state.kp_now:.0f}", (10, h - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

        # --- Lock progress bar (SEARCH / LOCK) ---
        if state in ("SEARCH", "LOCK"):
            lock_duration = 2.0  # TODO: read from param
            progress = min(1.0, self._latest_state.lock_elapsed_sec / lock_duration)
            bar_w = int(w * 0.4)
            bar_x, bar_y = 10, h - 20
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 10), (80, 80, 80), -1)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(bar_w * progress), bar_y + 10), color, -1)
            cv2.putText(frame, "LOCK", (bar_x, bar_y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # --- Error vector (LOCK / TRACK / FIRE) : 노란색 = 풍선 방향 ---
        if state in ("LOCK", "TRACK", "FIRE"):
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
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
