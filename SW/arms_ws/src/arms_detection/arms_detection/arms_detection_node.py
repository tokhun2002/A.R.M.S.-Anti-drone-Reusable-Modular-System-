#!/usr/bin/env python3
"""
arms_detection_node — A.R.M.S. 다중 검출기 (우선순위 기반)

검출기 우선순위: YOLO > HSV > ABSDIFF
  - YOLO    : 같은 프로세스에서 직접 추론(ultralytics). 모델/GPU 없으면 자동 비활성.
  - HSV     : 빨간색 영역 색상 분리 (근거리, 색이 선명할 때 강력)
  - ABSDIFF : 배경 대비 튀는 점 검출 (색·형태 무관, 원거리 소형 표적에 유리)

검출이 어느 정도 연속되면 CSRT/KCF 트래커로 전환(detect-then-track)해 TRACK
구간에선 ROI 만 추적한다(싸고 매끄러움). control 엔 완전히 투명 — 더 연속적인
/arms/detections 를 낼 뿐이다.

토픽
  구독 : /arms/image_raw        sensor_msgs/Image
  발행 : /arms/detections       arms_msgs/DetectionArray
  발행 : /arms/roi_image        sensor_msgs/Image  (bgr8, 추적 표적 확대 크롭)
  발행 : /arms/debug_image      sensor_msgs/Image  (bgr8, 모든 검출기 결과+최종 시각화)
  발행 : /arms/debug_absdiff    sensor_msgs/Image  (mono8, absdiff 이진 마스크)

파라미터 (ros2 param set /arms_detection_node ...)
  use_yolo/use_hsv/use_absdiff : bool  검출기 on/off (기본 true)
  yolo.acquire_interval : int   ACQUIRE/LOST 중 YOLO 를 N프레임마다만 실행 (기본 2).
                                TRACK 재검출은 항상 실행. 모델 환경변수 ARMS_MODEL/
                                ARMS_CONF/ARMS_IOU 로 설정(없으면 YOLO 비활성).
  proc_width            : int   검출 처리 가로 해상도, 0=원본 (기본 640)
  absdiff.diff_thresh   : int   배경 대비 임계값 (기본 25)
  absdiff.bg_blur       : int   배경 추정 Gaussian 커널, 홀수 (기본 15)
  absdiff.pre_blur      : int   노이즈 제거 Gaussian 커널, 홀수 (기본 1=끔)
  absdiff.max_area_ratio: float blob 최대 면적비 (기본 0.05)
  absdiff.max_blobs     : int   프레임 blob 수가 이보다 많으면 어수선한 장면(지상 등)
                                으로 보고 absdiff 결과 억제. 0=게이팅 끔 (기본 100)
  publish_debug         : bool  디버그 영상 발행 (기본 true)
  track.enable          : bool  detect-then-track on/off (기본 true, false=매프레임 검출)
  track.tracker_type    : str   "CSRT" | "KCF" (기본 CSRT)
  track.confirm_frames  : int   트래킹 시작 연속 검출 수 (기본 3)
  track.redetect_interval: int  TRACK 중 전체검출 보정 주기 (기본 10)
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
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from arms_msgs.msg import BoundingBox, DetectionArray


# ---------------------------------------------------------------------------
# Image conversion helpers (cv_bridge 없이)
# ---------------------------------------------------------------------------

_MORPH_KERNEL = np.ones((3, 3), np.uint8)


def _is_cv_detection(box) -> bool:
    """학습 모델이 아닌 고전 CV 검출 결과인지 판별한다."""
    return box is not None and box.class_name in {
        "hsv_red", "absdiff_spot", "cv_fused"
    }


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


def mono_to_imgmsg(gray: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = gray.shape[:2]
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = gray.shape[1]
    msg.data = gray.tobytes()
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

        # 고전 CV 후보는 크기가 작을수록 confidence가 낮아지는 특성 때문에 제어기의
        # 제어 임계값을 영원히 넘지 못했다. HSV와 고주파 점 검출이 합의한
        # cv_fused 후보만 여러 프레임 같은 위치에 있을 때 승격한다. 단일 CV 후보는
        # 상태표시/추적 단서로만 남겨 빨간 건물·조명에 즉시 LOCK 되는 것을 막는다.
        if _is_cv_detection(target):
            if (target.class_name == "cv_fused" and
                    self.hit_streak >= cfg["cv_confirm_frames"]):
                target.confidence = max(target.confidence, cfg["cv_confirm_conf"])
            elif (target.class_name == "hsv_red" and
                  self.hit_streak >= cfg["hsv_confirm_frames"]):
                target.confidence = max(target.confidence, cfg["hsv_confirm_conf"])
            else:
                target.confidence = min(target.confidence, 0.60)
        # 승격/제한을 적용한 confidence를 트래커에도 넘겨야 TRACK 상태에서 다시
        # 원래의 낮은 CV 점수로 떨어지지 않는다.
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
        self.declare_parameter("use_absdiff", True)
        # ACQUIRE/LOST(매프레임 전체탐색)에서 YOLO 를 N프레임마다만 실행해 레이트 유지.
        # TRACK 재검출은 드물게 일어나므로 항상 실행. (동기 추론)
        self.declare_parameter("yolo.acquire_interval", 2)
        self.declare_parameter("yolo.proposal_crop_px", 192)
        # 검출 처리 해상도(가로 px). 0=원본. 출력은 비율이라 정확도 무관, CV 비용만 절감.
        self.declare_parameter("proc_width", 640)
        self.declare_parameter("absdiff.diff_thresh",    25)
        self.declare_parameter("absdiff.bg_blur",        15)
        self.declare_parameter("absdiff.pre_blur",        1)
        self.declare_parameter("absdiff.max_area_ratio", 0.05)
        # blob 수가 이보다 많으면 어수선한 장면(지상 등)으로 보고 absdiff 억제. 0=끔.
        self.declare_parameter("absdiff.max_blobs", 100)
        self.declare_parameter("publish_debug", True)

        # --- CV 검출 confidence 게이팅 (표적 없을 때 헛-LOCK 방지) ---------------
        # CV(HSV/ABSDIFF)는 "가장 그럴듯한 blob"을 항상 하나 잡는다. 그래서 예전엔
        # confidence 바닥값(0.7/0.65)이 항상 임계값 이상이라 표적이 없어도 LOCK 됐다.
        # 이제 confidence 를 표적 '크기(프레임 대비 면적비)'에 비례시켜, 충분히 큰 =
        # 확실한 표적일 때만 임계값(state machine 의 confidence_threshold)을 넘게 한다.
        #   min_area_ratio      : 이보다 작은 blob 은 아예 검출로 치지 않음(존재 게이트)
        #   full_conf_area_ratio: 면적비가 이 값이면 confidence=1.0 (포화 기준)
        #   → 더 엄격히(큰 표적만) 하려면 full_conf_area_ratio 를 키우거나
        #     arms_control 의 mission.detection_confidence_threshold 를 올린다.
        self.declare_parameter("hsv.min_area_ratio",          0.00003)
        self.declare_parameter("hsv.full_conf_area_ratio",    0.0100)
        self.declare_parameter("hsv.max_area_ratio",          0.0200)
        # OpenCV hue(0..179)는 빨강이 0/179 경계에 걸쳐 있다. 경계 두 구간을
        # hard threshold로 자르는 대신 원형 hue 거리의 Gaussian 확률로 평가한다.
        self.declare_parameter("hsv.hue_sigma",               12.0)
        self.declare_parameter("hsv.prob_threshold",          0.20)
        self.declare_parameter("hsv.sat_center",              75.0)
        self.declare_parameter("hsv.sat_scale",               22.0)
        self.declare_parameter("hsv.val_min",                 30.0)
        self.declare_parameter("hsv.val_max_center",          225.0)
        self.declare_parameter("hsv.val_scale",               18.0)
        self.declare_parameter("hsv.min_circularity",         0.20)
        self.declare_parameter("hsv.texture_scale",           120.0)
        self.declare_parameter("absdiff.min_area_ratio",      0.00003)
        self.declare_parameter("absdiff.full_conf_area_ratio", 0.0100)
        self.declare_parameter("fusion.max_center_dist",      0.035)

        # 초기 획득도 CV를 쓰되, 여러 프레임의 위치 일치와 HSV/고주파 융합을 확인한
        # 뒤에만 제어 임계값 이상으로 승격한다. 원거리에서 YOLO가 놓친 표적을 위한
        # proposal을 만들면서 순간적인 빨간 조명에는 바로 LOCK하지 않게 한다.
        self.declare_parameter("acquire.yolo_only", False)
        self.declare_parameter("acquire.cv_confirm_frames", 5)
        self.declare_parameter("acquire.cv_confirm_conf", 0.72)
        self.declare_parameter("acquire.hsv_confirm_frames", 8)
        self.declare_parameter("acquire.hsv_confirm_conf", 0.68)

        # SEARCH 진단용: 전체 프레임 경로에서 세 검출기(YOLO/HSV/ABSDIFF) 각각의
        # 검출 여부·confidence 를 /arms/detector_status 로 발행해 UI 가 표시한다.
        # LOCK 판단(acquire.yolo_only)과 무관한 '표시 전용'이라, True 여도 HSV/ABSDIFF
        # 는 락을 만들지 않고 상태 보고만 한다. CPU 아끼려면 False.
        self.declare_parameter("debug.detector_status", True)

        # --- detect-then-track (ROI + CSRT/KCF) ---
        self.declare_parameter("track.enable", True)          # false=기존 매프레임 검출
        self.declare_parameter("track.tracker_type", "CSRT")  # "CSRT" | "KCF"
        self.declare_parameter("track.confirm_frames", 3)     # 트래킹 시작 연속 검출 수
        self.declare_parameter("track.confirm_dist", 0.035)   # 연속 판정 중심거리(norm)
        self.declare_parameter("track.redetect_interval", 10) # TRACK 중 재검출 주기
        self.declare_parameter("track.redetect_margin", 2.0)  # TRACK 재검출 ROI 크롭 확장배율
        self.declare_parameter("track.match_dist", 0.1)       # 재획득 예측 게이팅 거리
        self.declare_parameter("track.reacquire_frames", 8)   # LOST 재획득 창
        self.declare_parameter("track.max_unconfirmed", 3)    # TRACK 미확인 허용(드리프트 안전장치)
        self.declare_parameter("track.min_box_px", 4)         # 트래커 init 최소 박스
        self.declare_parameter("roi.margin", 1.8)             # ROI 크롭 확장 배율

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self._tracker = _TargetTracker()
        self._frame_i = 0   # YOLO ACQUIRE 스로틀용 프레임 카운터
        # 디버그 영상용: 이번 프레임 전체탐색(FULL)에서 각 검출기 원시 결과 보관
        #   {"yolo","hsv","absdiff","cv_fused"} → BoundingBox|None. _publish_debug 가 전부 그림.
        self._dbg_boxes = {}

        # YOLO: 같은 프로세스에서 직접 추론. ultralytics/모델이 없으면(호스트/SITL)
        # 조용히 비활성화하고 HSV/ABSDIFF 로만 동작 (graceful degradation).
        # 모델 경로/파라미터는 환경변수로 (컨테이너 compose 가 주입).
        self._yolo = None
        # 카메라 재학습 모델의 validation F1 최적점은 약 0.32다.
        self._yolo_conf = float(os.environ.get("ARMS_CONF", "0.32"))
        self._yolo_iou  = float(os.environ.get("ARMS_IOU",  "0.45"))
        self._yolo_imgsz = int(os.environ.get("ARMS_IMGSZ", "320"))
        # TensorRT 엔진은 첫 predict() 에서 지연 로딩된다. GPU OOM/버전불일치로 실패하면
        # 예외를 그대로 두면 노드가 죽고 재시작→CUDA 컨텍스트 미해제로 'device busy'
        # 폭주가 난다. 아래 상태로 "실패 시 크래시 대신 쿨다운 후 재시도"(그동안
        # HSV/ABSDIFF 폴백)하게 만든다.
        self._yolo_retry_after = 0.0   # 이 시각(monotonic)까지는 YOLO 재시도 안 함
        self._yolo_fail_count  = 0
        self._yolo_retry_cooldown = float(os.environ.get("ARMS_YOLO_RETRY_SEC", "3.0"))
        # 이 횟수만큼 연속 실패하면 YOLO 를 영구히 포기하고 CV(HSV/ABSDIFF)로만 돈다.
        #   → GPU OOM/엔진 불일치로 계속 실패할 때 재시도가 executor 를 블로킹해
        #     검출이 아예 안 나가는 문제를 막는다. 0 이하면 무한 재시도(기존 동작).
        self._yolo_max_fails = int(os.environ.get("ARMS_YOLO_MAX_FAILS", "3"))
        model_path = os.environ.get("ARMS_MODEL", "")
        if model_path:
            try:
                from ultralytics import YOLO
                self.get_logger().info(f"Loading YOLO model: {model_path}")
                self._yolo = YOLO(model_path, task="detect")
                self.get_logger().info("YOLO loaded (in-process).")
            except Exception as e:
                self.get_logger().warn(
                    f"YOLO disabled (load failed: {e}) → HSV/ABSDIFF only.")
        else:
            self.get_logger().info("ARMS_MODEL unset → YOLO disabled (HSV/ABSDIFF only).")

        self.create_subscription(Image, "/arms/image_raw", self._cb_image, qos)

        self.pub_det     = self.create_publisher(DetectionArray, "/arms/detections",    10)
        self.pub_debug   = self.create_publisher(Image,          "/arms/debug_image",   10)
        self.pub_absdiff = self.create_publisher(Image,          "/arms/debug_absdiff", 10)
        self.pub_roi     = self.create_publisher(Image,          "/arms/roi_image",     10)
        # 검출기 상태: [yolo, hsv, absdiff] 각 값 = 0~1 confidence,
        #   -1=off/미실행, -2=사용불가(YOLO OOM 등). UI 가 SEARCH 에서 표시.
        self.pub_detstatus = self.create_publisher(Float32MultiArray, "/arms/detector_status", 10)

        if not _tracker_available():
            self.get_logger().warn(
                "cv2 트래커(CSRT/KCF) 없음 — opencv-contrib 미설치. "
                "detect-then-track 비활성 → 매프레임 검출로 폴백(기능은 정상).")

        self.get_logger().info("arms_detection_node ready  [priority: YOLO > HSV > ABSDIFF, detect-then-track]")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _cb_image(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
            return   # 빈 프레임(image_publisher 루프 경계 등) → 스킵
        self._frame_i += 1
        self._dbg_boxes = {}   # 이번 프레임 검출기 결과 초기화(전체탐색 시 채워짐)
        try:
            bgr_full = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return
        if bgr_full is None or bgr_full.size == 0:
            return

        # 검출 처리용 다운스케일. 원본(bgr_full)은 ROI 크롭용으로 보관.
        proc_w = int(self.get_parameter("proc_width").value)
        if proc_w > 0 and bgr_full.shape[1] > proc_w:
            ph = int(bgr_full.shape[0] * proc_w / bgr_full.shape[1])
            bgr = cv2.resize(bgr_full, (proc_w, ph), interpolation=cv2.INTER_AREA)
        else:
            bgr = bgr_full

        # detect-then-track: FSM 이 검출/추적을 결정해 발행할 박스를 반환
        cfg = self._track_cfg()
        target = self._tracker.update(
            bgr,
            lambda img, roi=None: self._run_detectors(img, msg.header, roi),
            cfg)

        out = DetectionArray()
        out.header = msg.header
        if target is not None:
            out.detections.append(target)
        self.pub_det.publish(out)

        # ROI 확대 뷰 (유효 타깃 + 구독자 있을 때만) — full-res 에서 크롭
        if target is not None and self.pub_roi.get_subscription_count() > 0:
            self._publish_roi(bgr_full, target, msg.header,
                              float(self.get_parameter("roi.margin").value))

        # 디버그 이미지는 구독자가 있을 때만 그려서 발행 (없으면 발행 비용 전부 절감)
        if bool(self.get_parameter("publish_debug").value) \
                and self.pub_debug.get_subscription_count() > 0:
            self._publish_debug(bgr, msg.header, target, self._tracker.state)

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
            "cv_confirm_frames": int(g("acquire.cv_confirm_frames").value),
            "cv_confirm_conf":   float(g("acquire.cv_confirm_conf").value),
            "hsv_confirm_frames": int(g("acquire.hsv_confirm_frames").value),
            "hsv_confirm_conf":   float(g("acquire.hsv_confirm_conf").value),
        }

    def _run_detectors(self, bgr, header=None, roi_box=None):
        """YOLO > HSV > ABSDIFF 우선순위로 best 박스 하나를 반환 (없으면 None).
        roi_box 가 주어지면(TRACK 재검출) 그 주변만 크롭해 검출 후 full-frame 좌표로
        역변환한다 — 크롭이 작아 YOLO 가 싸고 소형 표적에 유리. YOLO 는 in-process 동기.
        전체 프레임 경로(ACQUIRE/LOST)에선 yolo.acquire_interval 마다만 YOLO 실행."""
        if roi_box is not None:
            margin = float(self.get_parameter("track.redetect_margin").value)
            crop, off = self._crop_roi(bgr, roi_box, margin)
            if crop is None:
                return None
            # ROI 재검출은 YOLO 항상 실행. absdiff 는 배경 문맥이 없어 무의미하므로
            # header=None(디버그 마스크 미발행). 우선순위 stack 은 동일.
            # LOCK/추적 모드(status_mode=1)로 상태 발행 → UI 가 ROI 안 confidence 를 본다.
            report = bool(self.get_parameter("debug.detector_status").value)
            box = self._detect_stack(crop, None, run_yolo=True, allow_cv=True,
                                     report_status=report, status_mode=1.0)
            if box is None:
                return None
            return self._remap_from_crop(box, off, bgr.shape[1], bgr.shape[0])

        # 초기 획득은 기본적으로 CV 후보도 함께 사용한다. HSV+고주파가 여러 프레임
        # 같은 위치에서 합의해야 confidence가 승격되므로 작은 원거리 풍선을 살리면서
        # 붉은 건물/조명의 순간 오검출은 억제한다. 필요하면 yolo_only로 되돌릴 수 있다.
        yolo_only = bool(self.get_parameter("acquire.yolo_only").value)
        allow_cv  = not yolo_only
        interval = max(1, int(self.get_parameter("yolo.acquire_interval").value))
        run_yolo = (self._frame_i % interval == 0)
        report = bool(self.get_parameter("debug.detector_status").value)
        return self._detect_stack(bgr, header, run_yolo=run_yolo, allow_cv=allow_cv,
                                  report_status=report, status_mode=0.0)  # FULL

    def _detect_stack(self, img, header, run_yolo, allow_cv=True,
                      report_status=False, status_mode=0.0):
        """단일 이미지에 대해 우선순위 검출 stack 실행 → best 박스(없으면 None).

        allow_cv=False 면 YOLO 로만 판단하고 HSV/ABSDIFF 폴백을 쓰지 않는다
        (초기 획득에서 CV 헛검출로 인한 오-LOCK 방지).
        report_status=True 면 세 검출기 결과를 /arms/detector_status 로 발행한다
        (표시 전용 — LOCK 판단에는 영향 없음). status_mode: 0=FULL(획득), 1=ROI(추적)."""
        yolo_on     = bool(self.get_parameter("use_yolo").value)
        use_yolo    = yolo_on and self._yolo is not None
        use_hsv     = bool(self.get_parameter("use_hsv").value)
        use_absdiff = bool(self.get_parameter("use_absdiff").value)

        yolo_box = self._detect_yolo(img) if (use_yolo and run_yolo) else None

        # CV 는 락에 필요하거나(폴백) 상태표시가 필요할 때 실행한다.
        need_cv     = report_status or (allow_cv and yolo_box is None)
        hsv_box     = self._detect_hsv(img)             if (use_hsv     and need_cv) else None
        absdiff_box = self._detect_absdiff(img, header, hsv_box) if (use_absdiff and need_cv) else None
        fused_box = self._fuse_cv(hsv_box, absdiff_box) if allow_cv else None

        # 전체 화면 YOLO가 놓친 작은 표적은 CV 융합 위치 주변의 고정 크기 ROI를
        # 잘라 다시 추론한다. ultralytics가 작은 crop을 입력 크기로 확대하므로
        # 원본에서 수 픽셀인 풍선도 학습 때의 ROI 샘플과 비슷한 크기로 보인다.
        if (yolo_box is None and use_yolo and run_yolo and fused_box is not None and
                status_mode == 0.0):
            yolo_box = self._detect_yolo_proposal(img, fused_box)

        if report_status:
            # 값: 0~1 confidence, -1=off/미실행, -2=사용불가(YOLO OOM 등)
            yv = (-1.0 if not yolo_on else
                  (-2.0 if self._yolo is None else
                   (-1.0 if not run_yolo else
                    (float(yolo_box.confidence) if yolo_box else 0.0))))
            hv = (-1.0 if not use_hsv     else (float(hsv_box.confidence)     if hsv_box     else 0.0))
            av = (-1.0 if not use_absdiff else (float(absdiff_box.confidence) if absdiff_box else 0.0))
            msg = Float32MultiArray()
            msg.data = [yv, hv, av, float(status_mode)]   # [yolo, hsv, absdiff, mode(0=FULL,1=ROI)]
            self.pub_detstatus.publish(msg)

        # 디버그 영상용: 전체탐색(FULL) 경로의 각 검출기 원시 결과를 보관(좌표가 proc
        # 프레임 정규화라 _publish_debug 가 그대로 그릴 수 있다). ROI 크롭 경로는 좌표계가
        # 달라 저장하지 않는다.
        if status_mode == 0.0:
            self._dbg_boxes = {"yolo": yolo_box, "hsv": hsv_box,
                               "absdiff": absdiff_box, "cv_fused": fused_box}

        # LOCK 판단용 winner. CV끼리는 같은 위치를 가리킬 때만 융합 후보로 승격한다.
        if yolo_box is not None:
            return yolo_box
        if not allow_cv:
            return None
        if fused_box is not None:
            return fused_box
        return hsv_box or absdiff_box

    def _detect_yolo_proposal(self, bgr, proposal):
        h, w = bgr.shape[:2]
        side = max(32, int(self.get_parameter("yolo.proposal_crop_px").value))
        cx, cy = int(proposal.x_center * w), int(proposal.y_center * h)
        half = side // 2
        x0, y0 = max(0, cx - half), max(0, cy - half)
        x1, y1 = min(w, x0 + side), min(h, y0 + side)
        x0, y0 = max(0, x1 - side), max(0, y1 - side)
        crop = bgr[y0:y1, x0:x1]
        if crop.size == 0:
            return None
        box = self._detect_yolo(crop)
        if box is None:
            return None
        return self._remap_from_crop(box, (x0, y0, x1 - x0, y1 - y0), w, h)

    def _fuse_cv(self, hsv_box, absdiff_box):
        if hsv_box is None or absdiff_box is None:
            return None
        dist = _center_dist((hsv_box.x_center, hsv_box.y_center),
                            (absdiff_box.x_center, absdiff_box.y_center))
        max_dist = float(self.get_parameter("fusion.max_center_dist").value)
        # 박스가 큰 경우에는 중심 허용거리도 박스 크기에 비례해 늘린다.
        adaptive = max(max_dist, 0.75 * max(hsv_box.width, hsv_box.height,
                                             absdiff_box.width, absdiff_box.height))
        if dist > adaptive:
            return None
        wh = max(0.1, float(hsv_box.confidence))
        wa = max(0.1, float(absdiff_box.confidence))
        den = wh + wa
        box = BoundingBox()
        box.x_center = float((wh * hsv_box.x_center + wa * absdiff_box.x_center) / den)
        box.y_center = float((wh * hsv_box.y_center + wa * absdiff_box.y_center) / den)
        box.width = float(max(hsv_box.width, absdiff_box.width))
        box.height = float(max(hsv_box.height, absdiff_box.height))
        # 이 값 자체로는 LOCK 임계값을 넘지 않는다. _TargetTracker가 시간 연속성을
        # 확인한 뒤 acquire.cv_confirm_conf로 승격한다.
        box.confidence = float(min(0.60, 0.5 * (wh + wa)))
        box.class_id = 3
        box.class_name = "cv_fused"
        return box

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

    def _detect_yolo(self, bgr: np.ndarray) -> BoundingBox | None:
        """같은 프로세스에서 YOLO 추론 → 최고 confidence 박스 하나 (normalized).

        엔진 로딩/추론이 GPU OOM 등으로 실패해도 노드를 죽이지 않는다. 실패하면
        쿨다운을 걸고 None(=이번 프레임 YOLO 스킵, HSV/ABSDIFF 폴백)을 반환하며,
        쿨다운 후 자동으로 다시 시도한다. 같은 프로세스 안에서 재시도하므로 재시작
        시 발생하던 'CUDA device busy(error 46)' 폭주가 생기지 않는다.
        """
        if self._yolo is None:
            return None
        now = time.monotonic()
        if now < self._yolo_retry_after:
            return None   # 최근 실패 → 쿨다운 동안 YOLO 건너뜀
        h, w = bgr.shape[:2]
        # Ultralytics의 ndarray 입력 규약은 OpenCV BGR이며 predictor 내부에서 RGB로
        # 변환한다. 여기서 미리 뒤집으면 R/B가 두 번 바뀌어 빨간 풍선 성능이 급락한다.
        source = np.ascontiguousarray(bgr)
        try:
            res = self._yolo.predict(source, conf=self._yolo_conf, iou=self._yolo_iou,
                                     imgsz=self._yolo_imgsz, verbose=False)
        except Exception as e:
            self._yolo_fail_count += 1
            # 반쯤 초기화된 backend/컨텍스트를 폐기 → 다음 시도에서 깨끗이 재로딩.
            try:
                self._yolo.predictor = None
            except Exception:
                pass
            # 임계치 이상 연속 실패 → YOLO 영구 포기, 이후 CV(HSV/ABSDIFF)로만 동작.
            if self._yolo_max_fails > 0 and self._yolo_fail_count >= self._yolo_max_fails:
                self._yolo = None   # 다음 프레임부터 _detect_yolo 는 즉시 None 반환(블로킹 없음)
                self.get_logger().error(
                    f"YOLO {self._yolo_fail_count}회 연속 실패 — 영구 비활성화하고 "
                    f"CV(HSV/ABSDIFF)로만 동작한다 (GPU OOM/엔진 불일치): {e}")
                return None
            self._yolo_retry_after = now + self._yolo_retry_cooldown
            self.get_logger().warning(
                f"YOLO 추론/엔진로딩 실패 #{self._yolo_fail_count}/{self._yolo_max_fails} "
                f"(GPU OOM 등) — {self._yolo_retry_cooldown:.0f}s 후 재시도, "
                f"그동안 HSV/ABSDIFF 폴백: {e}")
            return None
        # 이전에 실패한 적이 있으면 복구 로그 후 카운터 리셋.
        if self._yolo_fail_count:
            self.get_logger().info(
                f"YOLO 추론 복구됨 (실패 {self._yolo_fail_count}회 후 정상).")
            self._yolo_fail_count = 0
        if not res or len(res[0].boxes) == 0:
            return None
        best = max(res[0].boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = best.xyxy[0].tolist()
        box = BoundingBox()
        box.x_center  = float((x1 + x2) / 2 / w)
        box.y_center  = float((y1 + y2) / 2 / h)
        box.width     = float((x2 - x1) / w)
        box.height    = float((y2 - y1) / h)
        box.confidence = float(best.conf[0])
        box.class_id   = int(best.cls[0])
        box.class_name = str(self._yolo.names[int(best.cls[0])])
        return box

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

    def _detect_hsv(self, bgr: np.ndarray) -> BoundingBox | None:
        """Gaussian red score + 형태 점수로 붉은 원형 후보를 검출한다."""
        h, w = bgr.shape[:2]
        red_prob = self._red_probability(bgr)
        threshold = float(self.get_parameter("hsv.prob_threshold").value)
        mask = (red_prob >= threshold).astype(np.uint8) * 255
        # close는 작은 풍선 내부의 압축 노이즈 구멍만 메우며 작은 점을 지우지 않는다.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        min_frac  = float(self.get_parameter("hsv.min_area_ratio").value)
        max_frac  = float(self.get_parameter("hsv.max_area_ratio").value)
        full_frac = float(self.get_parameter("hsv.full_conf_area_ratio").value)
        min_circ = float(self.get_parameter("hsv.min_circularity").value)
        best = None
        best_score = -1.0
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
            margin = max(12, 2 * max(bw, bh))
            px0, py0 = max(0, x - margin), max(0, y - margin)
            px1, py1 = min(w, x + bw + margin), min(h, y + bh + margin)
            context = cv2.cvtColor(bgr[py0:py1, px0:px1], cv2.COLOR_BGR2GRAY)
            texture = float(cv2.Laplacian(context, cv2.CV_32F).var()) if context.size else 1e6
            texture_scale = max(1.0, float(self.get_parameter("hsv.texture_scale").value))
            smooth_score = 1.0 / (1.0 + texture / texture_scale)
            score = (color_score * (0.35 + 0.65 * shape_score) *
                     (0.5 + 0.5 * aspect) * (0.15 + 0.85 * smooth_score))
            if score > best_score:
                best_score = score
                best = (x, y, bw, bh, area_frac, color_score, shape_score, smooth_score)
        if best is None:
            return None
        x, y, bw, bh, area_frac, color_score, shape_score, smooth_score = best
        box = BoundingBox()
        box.x_center  = float((x + bw / 2) / w)
        box.y_center  = float((y + bh / 2) / h)
        box.width     = float(bw / w)
        box.height    = float(bh / h)
        # confidence = 표적이 클수록 1.0 에 근접(full_frac 에서 포화). 작으면 낮아
        # state machine 의 임계값을 못 넘어 LOCK 되지 않는다.
        size_score = min(1.0, area_frac / full_frac) if full_frac > 0 else 1.0
        box.confidence = float(min(0.60,
                                   0.45 * color_score + 0.35 * shape_score +
                                   0.10 * size_score + 0.10 * smooth_score))
        box.class_id  = 2
        box.class_name = "hsv_red"
        return box

    def _detect_absdiff(self, bgr: np.ndarray, header=None,
                        hint: BoundingBox | None = None) -> BoundingBox | None:
        """배경 대비 튀는 점 검출 (색·형태 무관)."""
        h, w = bgr.shape[:2]

        diff_thresh = int(self.get_parameter("absdiff.diff_thresh").value)
        max_area    = float(self.get_parameter("absdiff.max_area_ratio").value) * w * h
        bg_k = int(self.get_parameter("absdiff.bg_blur").value)
        pre  = int(self.get_parameter("absdiff.pre_blur").value)
        if bg_k % 2 == 0:
            bg_k += 1
        if pre % 2 == 0:
            pre += 1

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (pre, pre), 0)
        # 배경 추정: GaussianBlur (medianBlur(15)는 ~85ms 로 너무 느림 → ~6ms)
        bg   = cv2.GaussianBlur(gray, (bg_k, bg_k), 0)
        diff = cv2.absdiff(gray, bg)
        _, mask = cv2.threshold(diff, diff_thresh, 255, cv2.THRESH_BINARY)
        # 원거리 풍선은 수 픽셀뿐이라 MORPH_OPEN을 하면 표적 자체가 사라진다.
        # 작은 구멍만 닫고 아래 면적/클러터/색상 점수로 노이즈를 억제한다.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)

        if header is not None and self.pub_absdiff.get_subscription_count() > 0:
            self.pub_absdiff.publish(mono_to_imgmsg(mask, header))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 장면 클러터 게이팅: blob 이 너무 많으면 하늘이 아니라 어수선한 지상 장면으로
        # 보고 absdiff 결과를 통째로 버린다(빨간풍선 데모 등에서 오검출 억제).
        # YOLO/HSV 는 이 게이팅과 무관하게 우선 처리되므로 영향 없음.
        max_blobs = int(self.get_parameter("absdiff.max_blobs").value)
        if max_blobs > 0 and len(cnts) > max_blobs:
            return None

        min_area = float(self.get_parameter("absdiff.min_area_ratio").value) * w * h
        red_prob = self._red_probability(bgr)
        best, best_score = None, -1.0
        for c in cnts:
            x, y, bw, bh = cv2.boundingRect(c)
            a_px = bw * bh
            if a_px > max_area or a_px < min_area:       # 너무 크거나(배경) 너무 작은(노이즈) 점 제외
                continue
            patch = diff[max(0, y):y + bh, max(0, x):x + bw]
            if patch.size == 0:
                continue
            contrast = float(patch.max())
            red_score = float(red_prob[y:y + bh, x:x + bw].mean())
            aspect = min(bw, bh) / max(1.0, float(max(bw, bh)))
            perimeter = cv2.arcLength(c, True)
            contour_area = cv2.contourArea(c)
            circularity = (4.0 * np.pi * contour_area / (perimeter * perimeter)
                           if perimeter > 0 else 1.0)
            shape_score = max(float(circularity), float(aspect) * 0.65)
            # 색을 완전 필수로 하지는 않되, 붉고 둥근 고대비 점을 강하게 우선한다.
            score = (contrast * (a_px ** 0.3) *
                     (0.15 + 0.85 * red_score) *
                     (0.35 + 0.65 * shape_score))
            if hint is not None:
                cx = (x + bw / 2.0) / w
                cy = (y + bh / 2.0) / h
                dist = _center_dist((cx, cy), (hint.x_center, hint.y_center))
                proximity = float(np.exp(-0.5 * (dist / 0.06) ** 2))
                score *= 0.10 + 0.90 * proximity
            if score > best_score:
                best_score = score
                best = (x, y, bw, bh, contrast, red_score, shape_score)

        if best is None:
            return None
        x, y, bw, bh, contrast, red_score, shape_score = best
        box = BoundingBox()
        box.x_center  = float((x + bw / 2) / w)
        box.y_center  = float((y + bh / 2) / h)
        box.width     = float(bw / w)
        box.height    = float(bh / h)
        # confidence = 크기·대비 둘 다 커야 높다. 작은 점/약한 대비는 낮아 LOCK 못 넘긴다.
        full_frac = float(self.get_parameter("absdiff.full_conf_area_ratio").value)
        size_factor = min(1.0, (bw * bh) / (full_frac * w * h)) if full_frac > 0 else 1.0
        contrast_factor = contrast / 255.0
        box.confidence = float(min(0.60,
                                   0.35 * size_factor + 0.30 * contrast_factor +
                                   0.20 * red_score + 0.15 * shape_score))
        box.class_id  = 1
        box.class_name = "absdiff_spot"
        return box

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------

    # 검출기별 색 (BGR)
    _DBG_PALETTE = {
        "yolo":     (255, 200, 0),   # 하늘색
        "hsv":      (0, 0, 255),     # 빨강
        "absdiff":  (0, 220, 0),     # 초록
        "cv_fused": (255, 0, 255),   # 자홍
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
        """모든 검출기(YOLO/HSV/ABSDIFF/cv_fused) 결과를 색상별로 전부 그리고,
        최종 선택(winner)은 흰 굵은 박스로 강조한다. 우상단에 범례 표시."""
        img = bgr.copy()
        h, w = img.shape[:2]
        dbg = self._dbg_boxes or {}

        # 개별 검출기 원시 결과(얇게, 색상별)
        for key in ("yolo", "hsv", "absdiff", "cv_fused"):
            box = dbg.get(key)
            if box is not None:
                self._draw_dbg_box(img, box, self._DBG_PALETTE[key], 1)

        # 최종 선택 박스: 흰 굵은 테두리로 강조
        if target is not None:
            self._draw_dbg_box(img, target, (255, 255, 255), 2)

        # 우상단 범례: 각 검출기 confidence + 최종 선택
        font = cv2.FONT_HERSHEY_SIMPLEX
        yy = 20
        for key in ("yolo", "hsv", "absdiff", "cv_fused"):
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
