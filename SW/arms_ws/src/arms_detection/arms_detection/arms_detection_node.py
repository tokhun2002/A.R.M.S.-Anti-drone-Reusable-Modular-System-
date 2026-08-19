#!/usr/bin/env python3
"""
arms_detection_node — A.R.M.S. 빨간 풍선 검출기

검출 구조: HSV proposal → ROI YOLO verification
  - HSV     : Gaussian red score로 붉은 원형 후보를 최대 2개 제안.
  - ROI YOLO: 후보 crop들을 한 번의 batch 추론으로 확인. YOLO 승인 결과만 발행.
  - Full YOLO: HSV가 놓치는 경우를 위해 5프레임마다 전체 화면 fallback.

검출이 어느 정도 연속되면 CSRT/KCF 트래커로 전환(detect-then-track)해 TRACK
구간에선 ROI 만 추적한다(싸고 매끄러움). control 엔 완전히 투명 — 더 연속적인
/arms/detections 를 낼 뿐이다.

토픽
  구독 : /arms/image_raw        sensor_msgs/Image
  발행 : /arms/detections       arms_msgs/DetectionArray
  발행 : /arms/roi_image        sensor_msgs/Image  (bgr8, 추적 표적 확대 크롭)
  발행 : /arms/debug_image      sensor_msgs/Image  (bgr8, 모든 검출기 결과+최종 시각화)
  발행 : /arms/hsv_debug_image  sensor_msgs/Image  (bgr8, HSV 마스크와 후보 시각화)
  발행 : /arms/detector_status  std_msgs/Float32MultiArray (검출·처리시간 진단값)

파라미터 (ros2 param set /arms_detection_node ...)
  use_yolo/use_hsv             : bool  검출/후보 제안 on/off (기본 true)
  yolo.full_fallback_interval  : int   전체 화면 YOLO fallback 주기 (기본 5)
  proc_width            : int   검출 처리 가로 해상도, 0=원본 (기본 640)
  publish_debug         : bool  디버그 영상 발행 (기본 true)
  track.enable          : bool  detect-then-track on/off (기본 true, false=매프레임 검출)
  track.tracker_type    : str   "CSRT" | "KCF" (기본 CSRT)
  track.confirm_frames  : int   트래킹 시작 연속 검출 수 (기본 3)
  track.redetect_interval: int  TRACK 중 ROI YOLO 확인 주기 (기본 5)
  track.reacquire_frames: int   LOST 재획득 창 (기본 8)
  roi.margin            : float ROI 크롭 확장 배율 (기본 1.8)
"""

import os
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray

from arms_msgs.msg import BoundingBox, DetectionArray


# ---------------------------------------------------------------------------
# Image conversion helpers (cv_bridge 없이)
# ---------------------------------------------------------------------------

_MORPH_KERNEL = np.ones((3, 3), np.uint8)


class _CenterKalman:
    """정규화 영상 좌표에서 동작하는 등속도 Kalman 필터."""

    def __init__(self):
        self._kf = None

    def reset(self):
        self._kf = None

    def update(self, x: float, y: float):
        if self._kf is None:
            kf = cv2.KalmanFilter(4, 2)
            kf.transitionMatrix = np.array(
                [[1, 0, 1, 0], [0, 1, 0, 1],
                 [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
            kf.measurementMatrix = np.array(
                [[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
            kf.processNoiseCov = np.diag([2e-5, 2e-5, 8e-5, 8e-5]).astype(np.float32)
            kf.measurementNoiseCov = np.diag([3e-4, 3e-4]).astype(np.float32)
            kf.errorCovPost = np.eye(4, dtype=np.float32) * 1e-3
            kf.statePost = np.array([[x], [y], [0], [0]], np.float32)
            self._kf = kf
            return float(x), float(y)
        self._kf.predict()
        state = self._kf.correct(np.array([[x], [y]], np.float32))
        return float(state[0, 0]), float(state[1, 0])

    def predicted_center(self):
        if self._kf is None:
            return None
        state = self._kf.statePost
        return (float(state[0, 0] + state[2, 0]),
                float(state[1, 0] + state[3, 0]))


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    ch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding, 3)
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def bgr_to_imgmsg(bgr: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = bgr.shape[1] * 3
    msg.data = bgr.tobytes()
    return msg


# ---------------------------------------------------------------------------
# Detect-then-track FSM (detection 노드 내부, control 에 투명)
# ---------------------------------------------------------------------------

def _center_dist(a, b) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def _tracker_available() -> bool:
    """opencv(main 또는 legacy)에 CSRT/KCF 트래커가 있는지."""
    return (hasattr(cv2, "TrackerCSRT_create") or hasattr(cv2, "TrackerKCF_create")
            or (hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create")))


def _make_cv_tracker(kind: str):
    """opencv 버전/빌드에 따라 트래커가 main 모듈 또는 cv2.legacy 에 있음 — 둘 다 시도."""
    up = str(kind).upper()
    if up == "KCF":
        if hasattr(cv2, "TrackerKCF_create"):
            return cv2.TrackerKCF_create()
        return cv2.legacy.TrackerKCF_create()
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    return cv2.legacy.TrackerCSRT_create()


class _TargetTracker:
    """표적을 어느 정도 연속 검출하면 CSRT/KCF 트래커로 전환해 ROI만 추적한다.
    ACQUIRE → TRACK → LOST 3상태. 좌표는 normalized(0..1); cv2 트래커에는
    다운스케일 프레임 픽셀 bbox 로 변환해 전달. update() 가 발행할 박스를 반환
    (없으면 None). detect_fn 호출 여부를 FSM 이 결정 → 비용 절감의 핵심."""
    ACQUIRE, TRACK, LOST = "ACQUIRE", "TRACK", "LOST"

    def __init__(self):
        self.state = self.ACQUIRE
        self._cv = None
        self._box = None                 # 마지막 발행 박스 (BoundingBox)
        self._prev_center = None         # ACQUIRE 연속 판정용 직전 중심
        self._last_center = None         # 모션 예측용
        self._vel = (0.0, 0.0)           # 중심 속도 EMA (normalized/frame)
        self.hit_streak = 0
        self._since_redetect = 0
        self._unconfirmed = 0
        self._lost_count = 0
        self._last_conf = 0.0
        self._kalman = _CenterKalman()

    # ---- geometry (normalized <-> pixel) ----
    @staticmethod
    def _box_to_rect(box, W, H):
        bw = box.width * W
        bh = box.height * H
        return box.x_center * W - bw / 2.0, box.y_center * H - bh / 2.0, bw, bh

    @staticmethod
    def _rect_to_box(rect, W, H, conf, cls_id, cls_name):
        x, y, bw, bh = rect
        b = BoundingBox()
        b.x_center = float((x + bw / 2.0) / W)
        b.y_center = float((y + bh / 2.0) / H)
        b.width = float(bw / W)
        b.height = float(bh / H)
        b.confidence = float(conf)
        b.class_id = int(cls_id)
        b.class_name = str(cls_name)
        return b

    def _init_cv(self, bgr, box, cfg) -> bool:
        H, W = bgr.shape[:2]
        x, y, bw, bh = self._box_to_rect(box, W, H)
        x = max(0.0, min(x, W - 1.0))
        y = max(0.0, min(y, H - 1.0))
        bw = max(1.0, min(bw, W - x))
        bh = max(1.0, min(bh, H - y))
        if bw < cfg["min_box_px"] or bh < cfg["min_box_px"]:
            return False
        try:
            t = _make_cv_tracker(cfg["tracker_type"])
            t.init(bgr, (int(x), int(y), int(bw), int(bh)))
        except Exception:
            return False
        self._cv = t
        self._box = box
        return True

    def _update_motion(self, c):
        if self._last_center is not None:
            vx = c[0] - self._last_center[0]
            vy = c[1] - self._last_center[1]
            self._vel = (0.5 * vx + 0.5 * self._vel[0],
                         0.5 * vy + 0.5 * self._vel[1])
        self._last_center = c

    def _near_prediction(self, det, cfg) -> bool:
        predicted = self._kalman.predicted_center()
        if predicted is None and self._last_center is None:
            return True
        if predicted is not None:
            px, py = predicted
        else:
            px = self._last_center[0] + self._vel[0] * self._lost_count
            py = self._last_center[1] + self._vel[1] * self._lost_count
        radius = max(cfg["match_dist"] * 3.0, 0.15)
        return _center_dist((det.x_center, det.y_center), (px, py)) < radius

    def _smooth_center(self, box):
        sx, sy = self._kalman.update(box.x_center, box.y_center)
        box.x_center = float(np.clip(sx, 0.0, 1.0))
        box.y_center = float(np.clip(sy, 0.0, 1.0))
        return box

    def _to_acquire(self):
        self.state = self.ACQUIRE
        self._cv = None
        self.hit_streak = 0
        self._prev_center = None
        self._kalman.reset()

    def _to_lost(self):
        self.state = self.LOST
        self._cv = None
        self._lost_count = 0

    # ---- FSM ----
    def update(self, bgr, detect_fn, cfg):
        if not cfg["enable"]:
            return detect_fn(bgr)            # 트래킹 비활성 → 기존 매프레임 검출
        H, W = bgr.shape[:2]
        if self.state == self.TRACK:
            return self._do_track(bgr, detect_fn, cfg, W, H)
        if self.state == self.LOST:
            return self._do_lost(bgr, detect_fn, cfg)
        return self._do_acquire(bgr, detect_fn, cfg)

    def _do_acquire(self, bgr, detect_fn, cfg):
        target = detect_fn(bgr)
        if target is None:
            self.hit_streak = 0
            self._prev_center = None
            return None
        c = (target.x_center, target.y_center)
        if self._prev_center is not None and \
                _center_dist(c, self._prev_center) < cfg["confirm_dist"]:
            self.hit_streak += 1
        else:
            self.hit_streak = 1
        self._prev_center = c
        target = self._smooth_center(target)

        # detect_fn은 YOLO가 확인한 결과만 반환한다. HSV 후보나 시간 연속성만으로
        # confidence를 승격하지 않는다.
        self._last_conf = target.confidence
        if self.hit_streak >= cfg["confirm_frames"] and self._init_cv(bgr, target, cfg):
            self.state = self.TRACK
            self._since_redetect = 0
            self._unconfirmed = 0
            self._last_center = c
            self._vel = (0.0, 0.0)
        return target                        # ACQUIRE 중엔 실검출 그대로 발행

    def _do_track(self, bgr, detect_fn, cfg, W, H):
        ok, rect = self._cv.update(bgr)
        if not ok:
            self._to_lost()
            return None
        tracked = self._rect_to_box(rect, W, H, self._last_conf,
                                    self._box.class_id, self._box.class_name)
        tracked = self._smooth_center(tracked)
        self._update_motion((tracked.x_center, tracked.y_center))
        self._box = tracked
        self._since_redetect += 1
        if self._since_redetect >= cfg["redetect_interval"]:
            self._since_redetect = 0
            det = detect_fn(bgr, self._box)  # 트래킹 박스 주변 ROI 크롭 재검출(드리프트 보정)
            if det is not None:
                self._last_conf = det.confidence
                if self._init_cv(bgr, det, cfg):   # 검출로 스냅(재init)
                    self._box = det
                    tracked = det
                self._unconfirmed = 0
            else:
                self._unconfirmed += 1
                if self._unconfirmed >= cfg["max_unconfirmed"]:
                    self._to_lost()          # 확인 실패 누적 → 드리프트 의심, 상실
                    return None
        return tracked

    def _do_lost(self, bgr, detect_fn, cfg):
        self._lost_count += 1
        det = detect_fn(bgr)                 # 매프레임 재획득 시도
        if det is not None and self._near_prediction(det, cfg):
            det = self._smooth_center(det)
            self._last_conf = det.confidence
            if self._init_cv(bgr, det, cfg):
                self.state = self.TRACK
                self._since_redetect = 0
                self._unconfirmed = 0
                self._last_center = (det.x_center, det.y_center)
                return det
        if self._lost_count >= cfg["reacquire_frames"]:
            self._to_acquire()
        return None                          # LOST 중엔 발행 안 함 (control 이 홀드)


# ---------------------------------------------------------------------------

class ArmsDetectionNode(Node):
    def __init__(self):
        super().__init__("arms_detection_node")

        self.declare_parameter("use_yolo",   True)
        self.declare_parameter("use_hsv",    True)
        # HSV→ROI YOLO가 기본 경로다. HSV가 놓치는 색 변화에 대비해 전체 화면
        # YOLO를 낮은 주기로 실행한다. 0 이하면 full fallback을 끈다.
        self.declare_parameter("yolo.full_fallback_interval", 5)
        self.declare_parameter("yolo.proposal_crop_px", 192)
        # 검출 처리 해상도(가로 px). 0=원본. 출력은 비율이라 정확도 무관, CV 비용만 절감.
        self.declare_parameter("proc_width", 320)
        self.declare_parameter("publish_debug", True)

        # --- HSV proposal (단독 LOCK 금지) ------------------------------------
        self.declare_parameter("hsv.max_candidates",            2)
        # 질감 계산은 비교적 비싸므로 기본 형상/색 점수 상위 후보에만 적용한다.
        self.declare_parameter("hsv.texture_shortlist",          8)
        self.declare_parameter("hsv.min_area_ratio",          0.00003)
        self.declare_parameter("hsv.full_conf_area_ratio",    0.0100)
        self.declare_parameter("hsv.max_area_ratio",          0.0200)
        # OpenCV hue(0..179)는 빨강이 0/179 경계에 걸쳐 있다. 경계 두 구간을
        # hard threshold로 자르는 대신 원형 hue 거리의 Gaussian 확률로 평가한다.
        self.declare_parameter("hsv.hue_sigma",               12.0)
        # 핑크색 벽처럼 hue가 빨강과 가깝지만 채도가 낮은 배경을 억제하도록
        # 실기체 튜닝값을 기본값으로 사용한다.
        self.declare_parameter("hsv.prob_threshold",          0.35)
        self.declare_parameter("hsv.sat_center",             100.0)
        self.declare_parameter("hsv.sat_scale",               22.0)
        self.declare_parameter("hsv.val_min",                 30.0)
        self.declare_parameter("hsv.val_max_center",          225.0)
        self.declare_parameter("hsv.val_scale",               18.0)
        self.declare_parameter("hsv.min_circularity",         0.35)
        self.declare_parameter("hsv.texture_scale",           120.0)
        # SEARCH 진단용: YOLO/HSV 각각의
        # 검출 여부·confidence 를 /arms/detector_status 로 발행해 UI 가 표시한다.
        # HSV 값은 후보 품질일 뿐 LOCK 판단에는 쓰지 않는다. 메시지 호환을 위해
        # 세 번째(과거 ABSDIFF) 값은 항상 -1로 보낸다.
        self.declare_parameter("debug.detector_status", True)

        # --- detect-then-track (ROI + CSRT/KCF) ---
        # False = CSRT/KCF 추적 안 쓰고 TRACK 구간에서도 매 프레임 풀프레임 검출 방식
        #   (HSV proposal→ROI YOLO + full fallback)으로 표적을 다시 찾는다. 추적기
        #   드리프트로 표적을 "놓치는(락 풀리는)" 문제를 없앤다. True 로 되돌리면 복원.
        self.declare_parameter("track.enable", False)         # false=매프레임 풀프레임 검출(추적기 미사용)
        self.declare_parameter("track.tracker_type", "CSRT")  # "CSRT" | "KCF"
        self.declare_parameter("track.confirm_frames", 3)     # 트래킹 시작 연속 검출 수
        self.declare_parameter("track.confirm_dist", 0.035)   # 연속 판정 중심거리(norm)
        self.declare_parameter("track.redetect_interval", 5)  # TRACK 중 ROI YOLO 확인 주기
        self.declare_parameter("track.redetect_margin", 2.0)  # TRACK 재검출 ROI 크롭 확장배율
        self.declare_parameter("track.match_dist", 0.1)       # 재획득 예측 게이팅 거리
        self.declare_parameter("track.reacquire_frames", 8)   # LOST 재획득 창
        self.declare_parameter("track.max_unconfirmed", 2)    # ROI YOLO 연속 실패 허용 횟수
        self.declare_parameter("track.min_box_px", 4)         # 트래커 init 최소 박스
        self.declare_parameter("roi.margin", 1.8)             # ROI 크롭 확장 배율

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self._tracker = _TargetTracker()
        self._frame_i = 0   # YOLO ACQUIRE 스로틀용 프레임 카운터
        self._last_frame_started = None
        self._diag = None
        self._diag_hsv_mask = None
        self._diag_hsv_proposals = []
        # 원격 브랜치의 통합 디버그 영상 기능과 성능 진단을 함께 유지한다.
        self._dbg_boxes = {}

        # YOLO: 같은 프로세스에서 직접 추론. ultralytics/모델이 없으면(호스트/SITL)
        # 조용히 비활성화한다. HSV 단독 후보는 안전상 검출로 발행하지 않는다.
        # 모델 경로/파라미터는 환경변수로 (컨테이너 compose 가 주입).
        self._yolo = None
        # 카메라 재학습 모델의 validation F1 최적점은 약 0.32다.
        self._yolo_conf = float(os.environ.get("ARMS_CONF", "0.32"))
        self._yolo_iou  = float(os.environ.get("ARMS_IOU",  "0.45"))
        legacy_imgsz = os.environ.get("ARMS_IMGSZ")
        self._yolo_full_imgsz = int(os.environ.get(
            "ARMS_FULL_IMGSZ", legacy_imgsz or "512"))
        self._yolo_roi_imgsz = int(os.environ.get(
            "ARMS_ROI_IMGSZ", legacy_imgsz or "320"))
        # TensorRT 엔진은 첫 predict() 에서 지연 로딩된다. GPU OOM/버전불일치로 실패하면
        # 예외를 그대로 두면 노드가 죽고 재시작→CUDA 컨텍스트 미해제로 'device busy'
        # 폭주가 난다. 아래 상태로 "실패 시 크래시 대신 쿨다운 후 재시도"(그동안
        # 이번 프레임 검출을 보류하고 나중에 재시도하게 만든다.
        self._yolo_retry_after = 0.0   # 이 시각(monotonic)까지는 YOLO 재시도 안 함
        self._yolo_fail_count  = 0
        self._yolo_retry_cooldown = float(os.environ.get("ARMS_YOLO_RETRY_SEC", "3.0"))
        # 이 횟수만큼 연속 실패하면 YOLO를 영구히 비활성화한다.
        #   → GPU OOM/엔진 불일치로 계속 실패할 때 재시도가 executor 를 블로킹해
        #     검출이 아예 안 나가는 문제를 막는다. 0 이하면 무한 재시도(기존 동작).
        self._yolo_max_fails = int(os.environ.get("ARMS_YOLO_MAX_FAILS", "3"))
        model_path = os.environ.get("ARMS_MODEL", "")
        # 고정 batch TensorRT engine은 여러 ndarray 입력을 받지 못할 수 있다.
        # .pt 모델은 batch 1회, .engine은 호환성을 위해 후보별 순차 추론한다.
        self._yolo_batch_enabled = not model_path.lower().endswith(".engine")
        if model_path:
            try:
                from ultralytics import YOLO
                self.get_logger().info(f"Loading YOLO model: {model_path}")
                self._yolo = YOLO(model_path, task="detect")
                self.get_logger().info("YOLO loaded (in-process).")
            except Exception as e:
                self.get_logger().warn(
                    f"YOLO disabled (load failed: {e}); HSV proposals cannot LOCK.")
        else:
            self.get_logger().info("ARMS_MODEL unset → detection disabled (HSV proposal only).")

        # 입력 전송: ARMS_INPUT_COMPRESSED=1 이면 압축(CompressedImage)을 구독·디코드
        # → 컨테이너로 오는 UDP 데이터량 급감(조각 드롭↓). 아니면 raw Image.
        if os.environ.get("ARMS_INPUT_COMPRESSED", "0") == "1":
            self.create_subscription(CompressedImage, "/arms/image_raw/compressed",
                                     self._cb_compressed, qos)
            self.get_logger().info("입력: 압축(/arms/image_raw/compressed)")
        else:
            self.create_subscription(Image, "/arms/image_raw", self._cb_image, qos)

        self.pub_det     = self.create_publisher(DetectionArray, "/arms/detections",    10)
        self.pub_debug   = self.create_publisher(Image,          "/arms/debug_image",   10)
        self.pub_roi     = self.create_publisher(Image,          "/arms/roi_image",     10)
        self.pub_hsv_debug = self.create_publisher(
            Image, "/arms/hsv_debug_image", 10)
        # 상태 배열: [yolo, hsv proposal, legacy slot, mode]. legacy slot은 -1 고정.
        self.pub_detstatus = self.create_publisher(Float32MultiArray, "/arms/detector_status", 10)

        if not _tracker_available():
            self.get_logger().warn(
                "cv2 트래커(CSRT/KCF) 없음 — opencv-contrib 미설치. "
                "detect-then-track 비활성 → 매프레임 검출로 폴백(기능은 정상).")

        self.get_logger().info(
            "arms_detection_node ready [HSV proposals → batched ROI YOLO + full fallback]")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _cb_image(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
            return   # 빈 프레임(image_publisher 루프 경계 등) → 스킵
        try:
            bgr_full = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return
        self._process(bgr_full, msg.header)

    def _cb_compressed(self, msg):
        # 압축(JPEG/PNG) 입력 → BGR 디코드. 컨테이너로 오는 UDP 데이터량을 크게 줄여
        # image_raw(~1MB) 대비 조각 드롭이 거의 사라진다.
        if not msg.data:
            return
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            bgr_full = cv2.imdecode(arr, cv2.IMREAD_COLOR)   # → BGR
        except Exception as e:
            self.get_logger().warn(f"compressed decode failed: {e}")
            return
        if bgr_full is None:
            return
        self._process(bgr_full, msg.header)

    def _process(self, bgr_full, header):
        """raw/압축 공통 처리 경로. bgr_full(BGR ndarray)와 header 를 받는다."""
        frame_started = time.perf_counter()
        frame_interval_ms = (0.0 if self._last_frame_started is None else
                             (frame_started - self._last_frame_started) * 1000.0)
        self._last_frame_started = frame_started
        self._frame_i += 1
        self._diag = {
            "frame_interval_ms": frame_interval_ms,
            "resize_ms": 0.0, "hsv_ms": 0.0, "yolo_ms": 0.0,
            "tracker_ms": 0.0, "total_ms": 0.0,
            "hsv_count": 0, "yolo_inputs": 0, "yolo_accepted": 0,
            "yolo_ran": False, "full_frame": False, "mode": 0.0,
            "yolo_box": None, "hsv_box": None,
        }
        self._diag_hsv_mask = None
        self._diag_hsv_proposals = []
        self._dbg_boxes = {}
        if bgr_full is None or bgr_full.size == 0:
            return

        # 검출 처리용 다운스케일. 원본(bgr_full)은 ROI 크롭용으로 보관.
        resize_started = time.perf_counter()
        proc_w = int(self.get_parameter("proc_width").value)
        if proc_w > 0 and bgr_full.shape[1] > proc_w:
            ph = int(bgr_full.shape[0] * proc_w / bgr_full.shape[1])
            bgr = cv2.resize(bgr_full, (proc_w, ph), interpolation=cv2.INTER_AREA)
        else:
            bgr = bgr_full
        self._diag["resize_ms"] = (time.perf_counter() - resize_started) * 1000.0

        # detect-then-track: FSM 이 검출/추적을 결정해 발행할 박스를 반환
        cfg = self._track_cfg()
        tracker_started = time.perf_counter()
        target = self._tracker.update(
            bgr,
            lambda img, roi=None: self._run_detectors(img, header, roi),
            cfg)
        update_ms = (time.perf_counter() - tracker_started) * 1000.0
        self._diag["tracker_ms"] = max(
            0.0, update_ms - self._diag["hsv_ms"] - self._diag["yolo_ms"])
        self._diag["total_ms"] = (time.perf_counter() - frame_started) * 1000.0
        self._publish_detector_status()

        if self.pub_hsv_debug.get_subscription_count() > 0:
            # TRACK/ROI 구간에는 원래 HSV를 실행하지 않는다. RQT 또는 UI의 S키
            # 일회성 캡처가 구독한 경우에만 HSV 화면을 별도로 한 번 계산한다.
            if self._diag_hsv_mask is None:
                self._diag_hsv_proposals = self._detect_hsv_candidates(bgr)
            self._publish_hsv_debug(bgr, header)

        out = DetectionArray()
        out.header = header
        if target is not None:
            out.detections.append(target)
        self.pub_det.publish(out)

        # ROI 확대 뷰 (유효 타깃 + 구독자 있을 때만) — full-res 에서 크롭
        if target is not None and self.pub_roi.get_subscription_count() > 0:
            self._publish_roi(bgr_full, target, header,
                              float(self.get_parameter("roi.margin").value))

        # 디버그 이미지는 구독자가 있을 때만 그려서 발행 (없으면 발행 비용 전부 절감)
        if bool(self.get_parameter("publish_debug").value) \
                and self.pub_debug.get_subscription_count() > 0:
            self._publish_debug(bgr, header, target, self._tracker.state)

    def _track_cfg(self) -> dict:
        g = self.get_parameter
        return {
            "enable":            bool(g("track.enable").value),
            "tracker_type":      str(g("track.tracker_type").value),
            "confirm_frames":    int(g("track.confirm_frames").value),
            "confirm_dist":      float(g("track.confirm_dist").value),
            "redetect_interval": int(g("track.redetect_interval").value),
            "match_dist":        float(g("track.match_dist").value),
            "reacquire_frames":  int(g("track.reacquire_frames").value),
            "max_unconfirmed":   int(g("track.max_unconfirmed").value),
            "min_box_px":        int(g("track.min_box_px").value),
        }

    def _run_detectors(self, bgr, header=None, roi_box=None):
        """획득은 HSV→ROI YOLO, 추적 재검증은 ROI YOLO로 실행한다."""
        if roi_box is not None:
            self._diag["mode"] = 1.0
            margin = float(self.get_parameter("track.redetect_margin").value)
            crop, off = self._crop_roi(bgr, roi_box, margin)
            if crop is None:
                return None
            crop_box = self._detect_yolo(crop, imgsz=self._yolo_roi_imgsz)
            if crop_box is None:
                return None
            box = self._remap_from_crop(
                crop_box, off, bgr.shape[1], bgr.shape[0])
            self._diag["yolo_box"] = box
            self._dbg_boxes = {"yolo": box, "hsv": None}
            return box

        return self._detect_acquire(bgr)

    def _detect_acquire(self, bgr):
        """HSV 후보를 먼저 만들고 ROI들을 한 번의 YOLO batch로 검증한다."""
        self._diag["mode"] = 0.0
        yolo_on = bool(self.get_parameter("use_yolo").value)
        use_yolo = yolo_on and self._yolo is not None
        use_hsv = bool(self.get_parameter("use_hsv").value)
        hsv_started = time.perf_counter()
        proposals = self._detect_hsv_candidates(bgr) if use_hsv else []
        self._diag["hsv_ms"] += (time.perf_counter() - hsv_started) * 1000.0
        self._diag["hsv_count"] = len(proposals)
        self._diag_hsv_proposals = proposals
        hsv_box = proposals[0] if proposals else None
        self._diag["hsv_box"] = hsv_box

        yolo_ran = False
        verified = []
        if use_yolo and proposals:
            yolo_ran = True
            verified = self._detect_yolo_proposals_batch(bgr, proposals)

        winner = (max(verified, key=lambda box: float(box.confidence))
                  if verified else None)

        # HSV가 색 변화로 후보를 놓치거나 ROI YOLO가 거부한 경우를 위한 저주기
        # 전체 화면 fallback. ROI가 성공한 프레임에는 중복 추론하지 않는다.
        interval = int(self.get_parameter("yolo.full_fallback_interval").value)
        full_due = interval > 0 and self._frame_i % interval == 0
        if winner is None and use_yolo and full_due:
            yolo_ran = True
            self._diag["full_frame"] = True
            winner = self._detect_yolo(bgr, imgsz=self._yolo_full_imgsz)

        self._diag["yolo_box"] = winner
        self._dbg_boxes = {"yolo": winner, "hsv": hsv_box}
        return winner

    def _detect_yolo_proposals_batch(self, bgr, proposals):
        """상위 HSV 후보 crop을 YOLO batch 1회로 검증해 원본 좌표로 돌린다."""
        h, w = bgr.shape[:2]
        side = max(32, int(self.get_parameter("yolo.proposal_crop_px").value))
        max_candidates = max(1, int(
            self.get_parameter("hsv.max_candidates").value))
        crops, offsets = [], []
        for proposal in proposals[:max_candidates]:
            cx, cy = int(proposal.x_center * w), int(proposal.y_center * h)
            half = side // 2
            x0, y0 = max(0, cx - half), max(0, cy - half)
            x1, y1 = min(w, x0 + side), min(h, y0 + side)
            x0, y0 = max(0, x1 - side), max(0, y1 - side)
            crop = bgr[y0:y1, x0:x1]
            if crop.size:
                crops.append(np.ascontiguousarray(crop))
                offsets.append((x0, y0, x1 - x0, y1 - y0))
        boxes = self._detect_yolo_batch(crops, imgsz=self._yolo_roi_imgsz)
        return [
            self._remap_from_crop(box, off, w, h)
            for box, off in zip(boxes, offsets) if box is not None
        ]

    def _publish_detector_status(self):
        if not bool(self.get_parameter("debug.detector_status").value):
            return
        diag = self._diag
        yolo_box = diag["yolo_box"]
        hsv_box = diag["hsv_box"]
        yolo_ran = bool(diag["yolo_ran"])
        yolo_on = bool(self.get_parameter("use_yolo").value)
        use_hsv = bool(self.get_parameter("use_hsv").value)
        yv = (-1.0 if not yolo_on else
              (-2.0 if self._yolo is None else
               (-1.0 if not yolo_ran else
                (float(yolo_box.confidence) if yolo_box else 0.0))))
        hv = (-1.0 if not use_hsv else
              (float(hsv_box.confidence) if hsv_box else 0.0))
        msg = Float32MultiArray()
        state_code = {"ACQUIRE": 0.0, "TRACK": 1.0, "LOST": 2.0}.get(
            self._tracker.state, -1.0)
        # 0~3은 기존 UI와 호환. 4 이후는 실시간 성능 진단용이다.
        msg.data = [
            yv, hv, -1.0, float(diag["mode"]),
            float(diag["hsv_count"]), float(diag["yolo_inputs"]),
            float(diag["yolo_accepted"]), float(diag["frame_interval_ms"]),
            float(diag["resize_ms"]), float(diag["hsv_ms"]),
            float(diag["yolo_ms"]), float(diag["tracker_ms"]),
            float(diag["total_ms"]), state_code, float(self._frame_i),
            1.0 if diag["full_frame"] else 0.0,
        ]
        self.pub_detstatus.publish(msg)

    @staticmethod
    def _crop_roi(bgr, box, margin):
        """정규화 box 주변을 margin 배 확장해 크롭. (crop, (x0,y0,cw,ch)) 반환."""
        H, W = bgr.shape[:2]
        bw = box.width * W * margin
        bh = box.height * H * margin
        cx, cy = box.x_center * W, box.y_center * H
        x0 = int(max(0, cx - bw / 2)); y0 = int(max(0, cy - bh / 2))
        x1 = int(min(W, cx + bw / 2)); y1 = int(min(H, cy + bh / 2))
        if x1 - x0 < 8 or y1 - y0 < 8:
            return None, None
        return bgr[y0:y1, x0:x1], (x0, y0, x1 - x0, y1 - y0)

    @staticmethod
    def _remap_from_crop(box, off, W, H):
        """크롭 좌표계 normalized box → full-frame normalized box."""
        x0, y0, cw, ch = off
        b = BoundingBox()
        b.x_center = float((x0 + box.x_center * cw) / W)
        b.y_center = float((y0 + box.y_center * ch) / H)
        b.width    = float(box.width  * cw / W)
        b.height   = float(box.height * ch / H)
        b.confidence = box.confidence
        b.class_id   = box.class_id
        b.class_name = box.class_name
        return b

    def _publish_roi(self, bgr_full, box, header, margin):
        H, W = bgr_full.shape[:2]
        bw = box.width * W * margin
        bh = box.height * H * margin
        cx, cy = box.x_center * W, box.y_center * H
        x1 = int(max(0, cx - bw / 2)); y1 = int(max(0, cy - bh / 2))
        x2 = int(min(W, cx + bw / 2)); y2 = int(min(H, cy + bh / 2))
        if x2 - x1 < 2 or y2 - y1 < 2:
            return
        crop = bgr_full[y1:y2, x1:x2]
        if crop.size == 0:
            return
        self.pub_roi.publish(bgr_to_imgmsg(np.ascontiguousarray(crop), header))

    # ------------------------------------------------------------------
    # Detectors
    # ------------------------------------------------------------------

    def _detect_yolo(self, bgr: np.ndarray,
                     imgsz: int | None = None) -> BoundingBox | None:
        """단일 이미지 YOLO 추론 결과의 최고 confidence 박스를 반환한다."""
        boxes = self._detect_yolo_batch([bgr], imgsz=imgsz)
        return boxes[0] if boxes else None

    def _detect_yolo_batch(self, images: list[np.ndarray],
                           imgsz: int | None = None) -> list[BoundingBox | None]:
        """여러 ROI를 한 번의 YOLO predict 호출로 검증한다.

        엔진 로딩/추론이 GPU OOM 등으로 실패해도 노드를 죽이지 않는다. 실패하면
        쿨다운을 걸고 해당 batch를 모두 miss로 처리하며,
        쿨다운 후 자동으로 다시 시도한다. 같은 프로세스 안에서 재시도하므로 재시작
        시 발생하던 'CUDA device busy(error 46)' 폭주가 생기지 않는다.
        """
        if not images:
            return []
        if self._yolo is None:
            return [None] * len(images)
        now = time.monotonic()
        if now < self._yolo_retry_after:
            return [None] * len(images)

        # TensorRT 고정 batch 엔진은 batch 입력 대신 순차 실행한다. 기본 .pt 경로는
        # 아래 list 입력으로 ROI 1~2개를 단일 batch 추론한다.
        if len(images) > 1 and not self._yolo_batch_enabled:
            return [self._detect_yolo(image, imgsz=imgsz) for image in images]

        sources = [np.ascontiguousarray(image) for image in images]
        input_size = int(imgsz or self._yolo_roi_imgsz)
        if self._diag is not None:
            self._diag["yolo_ran"] = True
            self._diag["yolo_inputs"] += len(sources)
        yolo_started = time.perf_counter()
        try:
            results = self._yolo.predict(
                sources, conf=self._yolo_conf, iou=self._yolo_iou,
                imgsz=input_size, verbose=False)
        except Exception as e:
            if self._diag is not None:
                self._diag["yolo_ms"] += (
                    time.perf_counter() - yolo_started) * 1000.0
            self._yolo_fail_count += 1
            # 반쯤 초기화된 backend/컨텍스트를 폐기 → 다음 시도에서 깨끗이 재로딩.
            try:
                self._yolo.predictor = None
            except Exception:
                pass
            # 임계치 이상 연속 실패 → YOLO 영구 비활성화. HSV는 proposal-only라
            # 안전상 검출 결과를 단독 발행하지 않는다.
            if self._yolo_max_fails > 0 and self._yolo_fail_count >= self._yolo_max_fails:
                self._yolo = None
                self.get_logger().error(
                    f"YOLO {self._yolo_fail_count}회 연속 실패 — 검출을 영구 "
                    f"비활성화한다 (GPU OOM/엔진 불일치): {e}")
                return [None] * len(images)
            self._yolo_retry_after = now + self._yolo_retry_cooldown
            self.get_logger().warning(
                f"YOLO 추론/엔진로딩 실패 #{self._yolo_fail_count}/{self._yolo_max_fails} "
                f"(GPU OOM 등) — {self._yolo_retry_cooldown:.0f}s 후 재시도, "
                f"그동안 검출 보류: {e}")
            return [None] * len(images)
        if self._diag is not None:
            self._diag["yolo_ms"] += (
                time.perf_counter() - yolo_started) * 1000.0
        # 이전에 실패한 적이 있으면 복구 로그 후 카운터 리셋.
        if self._yolo_fail_count:
            self.get_logger().info(
                f"YOLO 추론 복구됨 (실패 {self._yolo_fail_count}회 후 정상).")
            self._yolo_fail_count = 0
        output = []
        for image, result in zip(images, results):
            h, w = image.shape[:2]
            if len(result.boxes) == 0:
                output.append(None)
                continue
            best = max(result.boxes, key=lambda box: float(box.conf[0]))
            x1, y1, x2, y2 = best.xyxy[0].tolist()
            box = BoundingBox()
            box.x_center = float((x1 + x2) / 2 / w)
            box.y_center = float((y1 + y2) / 2 / h)
            box.width = float((x2 - x1) / w)
            box.height = float((y2 - y1) / h)
            box.confidence = float(best.conf[0])
            box.class_id = int(best.cls[0])
            box.class_name = str(self._yolo.names[int(best.cls[0])])
            output.append(box)
        if self._diag is not None:
            self._diag["yolo_accepted"] += sum(box is not None for box in output)
        # 비정상적으로 result 수가 적어도 호출자 zip 좌표가 어긋나지 않게 채운다.
        output.extend([None] * (len(images) - len(output)))
        return output

    def _red_probability(self, bgr: np.ndarray) -> np.ndarray:
        """빨강에 대한 0..1 soft score. Hue는 원형 Gaussian으로 계산한다."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
        hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        hue_dist = np.minimum(hue, 180.0 - hue)
        sigma = max(1.0, float(self.get_parameter("hsv.hue_sigma").value))
        hue_score = np.exp(-0.5 * (hue_dist / sigma) ** 2)
        sat_center = float(self.get_parameter("hsv.sat_center").value)
        sat_scale = max(1.0, float(self.get_parameter("hsv.sat_scale").value))
        sat_score = 1.0 / (1.0 + np.exp(-(sat - sat_center) / sat_scale))
        val_min = float(self.get_parameter("hsv.val_min").value)
        val_max = float(self.get_parameter("hsv.val_max_center").value)
        val_scale = max(1.0, float(self.get_parameter("hsv.val_scale").value))
        val_low_score = 1.0 / (1.0 + np.exp(-(val - val_min) / 12.0))
        # 완전히 포화된 LED/건물 조명은 풍선보다 약하게 평가하되 hard cut은 하지 않는다.
        val_high_score = 1.0 / (1.0 + np.exp((val - val_max) / val_scale))
        return (hue_score * sat_score * val_low_score * val_high_score).astype(np.float32)

    def _detect_hsv_candidates(self, bgr: np.ndarray) -> list[BoundingBox]:
        """붉은 원형 후보를 점수순으로 반환한다. 후보 자체는 검출이 아니다."""
        h, w = bgr.shape[:2]
        red_prob = self._red_probability(bgr)
        threshold = float(self.get_parameter("hsv.prob_threshold").value)
        mask = (red_prob >= threshold).astype(np.uint8) * 255
        # close는 작은 풍선 내부의 압축 노이즈 구멍만 메우며 작은 점을 지우지 않는다.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
        self._diag_hsv_mask = mask
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return []
        min_frac  = float(self.get_parameter("hsv.min_area_ratio").value)
        max_frac  = float(self.get_parameter("hsv.max_area_ratio").value)
        full_frac = float(self.get_parameter("hsv.full_conf_area_ratio").value)
        min_circ = float(self.get_parameter("hsv.min_circularity").value)
        preliminary = []
        for c in cnts:
            x, y, bw, bh = cv2.boundingRect(c)
            area_px = float(bw * bh)
            area_frac = area_px / float(w * h)
            if area_frac < min_frac or area_frac > max_frac:
                continue
            aspect = min(bw, bh) / max(1.0, float(max(bw, bh)))
            perimeter = cv2.arcLength(c, True)
            contour_area = cv2.contourArea(c)
            circularity = (4.0 * np.pi * contour_area / (perimeter * perimeter)
                           if perimeter > 0 else 1.0)
            # 1~2px 원거리 점은 contourArea가 0이 될 수 있어 aspect로 대신 평가한다.
            shape_score = max(float(circularity), float(aspect) * 0.65)
            if shape_score < min_circ:
                continue
            color_score = float(red_prob[y:y + bh, x:x + bw].mean())
            # 비싼 Laplacian 계산 전에 색·형상만으로 shortlist를 만든다.
            base_score = (color_score * (0.35 + 0.65 * shape_score) *
                          (0.5 + 0.5 * aspect))
            preliminary.append((base_score, x, y, bw, bh, area_frac,
                                color_score, shape_score, aspect))

        if not preliminary:
            return []
        preliminary.sort(key=lambda item: item[0], reverse=True)
        shortlist = max(
            int(self.get_parameter("hsv.texture_shortlist").value),
            int(self.get_parameter("hsv.max_candidates").value))
        candidates = []
        for (base_score, x, y, bw, bh, area_frac,
             color_score, shape_score, aspect) in preliminary[:max(1, shortlist)]:
            margin = max(12, 2 * max(bw, bh))
            px0, py0 = max(0, x - margin), max(0, y - margin)
            px1, py1 = min(w, x + bw + margin), min(h, y + bh + margin)
            context = cv2.cvtColor(bgr[py0:py1, px0:px1], cv2.COLOR_BGR2GRAY)
            texture = float(cv2.Laplacian(context, cv2.CV_32F).var()) if context.size else 1e6
            texture_scale = max(1.0, float(self.get_parameter("hsv.texture_scale").value))
            smooth_score = 1.0 / (1.0 + texture / texture_scale)
            score = base_score * (0.15 + 0.85 * smooth_score)
            box = BoundingBox()
            box.x_center = float((x + bw / 2) / w)
            box.y_center = float((y + bh / 2) / h)
            box.width = float(bw / w)
            box.height = float(bh / h)
            size_score = min(1.0, area_frac / full_frac) if full_frac > 0 else 1.0
            box.confidence = float(min(
                0.60, 0.45 * color_score + 0.35 * shape_score +
                0.10 * size_score + 0.10 * smooth_score))
            box.class_id = 2
            box.class_name = "hsv_proposal"
            candidates.append((score, box))
        candidates.sort(key=lambda item: item[0], reverse=True)
        max_candidates = max(1, int(
            self.get_parameter("hsv.max_candidates").value))
        return [box for _, box in candidates[:max_candidates]]

    def _publish_hsv_debug(self, bgr, header):
        """HSV 확률 마스크와 후보 박스를 작은 진단 영상으로 발행한다."""
        mask = self._diag_hsv_mask
        if mask is None:
            return
        img = bgr.copy()
        red_overlay = np.zeros_like(img)
        red_overlay[:, :, 2] = mask
        img = cv2.addWeighted(img, 0.75, red_overlay, 0.35, 0.0)
        h, w = img.shape[:2]
        for index, box in enumerate(self._diag_hsv_proposals, start=1):
            cx, cy = int(box.x_center * w), int(box.y_center * h)
            bw, bh = int(box.width * w), int(box.height * h)
            x1, y1 = cx - bw // 2, cy - bh // 2
            x2, y2 = cx + bw // 2, cy + bh // 2
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(img, f"H{index}", (x1, max(14, y1 - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"HSV candidates: {len(self._diag_hsv_proposals)}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2, cv2.LINE_AA)
        # 진단 패널용이므로 전송량을 제한한다.
        if w > 480:
            out_h = max(1, int(h * 480 / w))
            img = cv2.resize(img, (480, out_h), interpolation=cv2.INTER_AREA)
        self.pub_hsv_debug.publish(bgr_to_imgmsg(np.ascontiguousarray(img), header))

    def _detect_hsv(self, bgr: np.ndarray) -> BoundingBox | None:
        """진단/UI 호환용 최고 HSV proposal 하나를 반환한다."""
        candidates = self._detect_hsv_candidates(bgr)
        return candidates[0] if candidates else None

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------

    # 검출기별 색 (BGR)
    _DBG_PALETTE = {
        "yolo":     (255, 200, 0),   # 하늘색
        "hsv":      (0, 0, 255),     # 빨강
    }

    @staticmethod
    def _draw_dbg_box(img, box, color, thick):
        """정규화 box 를 img 에 사각형으로 그린다(너무 작으면 최소 크기 보장)."""
        h, w = img.shape[:2]
        cx, cy = box.x_center * w, box.y_center * h
        bw, bh = box.width * w, box.height * h
        x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
        x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
        if x2 - x1 < 6:
            x1, x2 = int(cx - 8), int(cx + 8)
        if y2 - y1 < 6:
            y1, y2 = int(cy - 8), int(cy + 8)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thick)

    def _publish_debug(self, bgr, header, target, state):
        """YOLO/HSV 결과와 최종 선택을 함께 표시한다."""
        img = bgr.copy()
        h, w = img.shape[:2]
        dbg = self._dbg_boxes or {}

        # 개별 검출기 원시 결과(얇게, 색상별)
        for key in ("yolo", "hsv"):
            box = dbg.get(key)
            if box is not None:
                self._draw_dbg_box(img, box, self._DBG_PALETTE[key], 1)

        # 최종 선택 박스: 흰 굵은 테두리로 강조
        if target is not None:
            self._draw_dbg_box(img, target, (255, 255, 255), 2)

        # 우상단 범례: 각 검출기 confidence + 최종 선택
        font = cv2.FONT_HERSHEY_SIMPLEX
        yy = 20
        for key in ("yolo", "hsv"):
            box = dbg.get(key)
            txt = f"{key}: {box.confidence:.2f}" if box is not None else f"{key}: -"
            col = self._DBG_PALETTE[key] if box is not None else (120, 120, 120)
            cv2.putText(img, txt, (w - 200, yy), font, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, txt, (w - 200, yy), font, 0.5, col, 1, cv2.LINE_AA)
            yy += 20
        if target is not None:
            t = f"FINAL {target.class_name} {target.confidence:.2f}"
            cv2.putText(img, t, (w - 260, yy), font, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, t, (w - 260, yy), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        # 중앙 마커 + 트래커 상태 (TRACK=초록, 그 외=주황)
        state_col = (0, 220, 0) if state == _TargetTracker.TRACK else (0, 165, 255)
        cv2.drawMarker(img, (w // 2, h // 2), (180, 180, 180),
                       cv2.MARKER_TILTED_CROSS, 16, 1)
        cv2.putText(img, state, (10, 24), font, 0.7, state_col, 2, cv2.LINE_AA)

        self.pub_debug.publish(bgr_to_imgmsg(img, header))


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ArmsDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
