#!/usr/bin/env python3
"""
arms_detection_node — A.R.M.S. 빨간 풍선 검출기

검출 구조: HSV proposal → ROI YOLO verification (매 프레임 전체 검출)
  - HSV     : Gaussian red score로 붉은 원형 후보를 제안.
  - ROI YOLO: 후보 crop들을 batch 추론으로 확인. YOLO 승인 결과만 발행.
  - Full YOLO: HSV가 놓치는 경우를 위해 전체 화면 fallback.

검출 결과는 칼만 필터로 평활하고, 표적을 확정한 뒤에는 예측 위치에서 크게 튄
검출(오인식)을 걸러낸다. 순간 놓침은 KF 예측으로 잠시 외삽(coast)한다. control
엔 완전히 투명 — 더 연속적인 /arms/detections 를 낼 뿐이다.

토픽
  구독 : /arms/image_raw        sensor_msgs/Image
  발행 : /arms/detections       arms_msgs/DetectionArray
  발행 : /arms/roi_image        sensor_msgs/Image  (bgr8, 표적 확대 크롭)
  발행 : /arms/debug_image      sensor_msgs/Image  (bgr8, 모든 검출기 결과+최종 시각화)
  발행 : /arms/hsv_debug_image  sensor_msgs/Image  (bgr8, HSV 마스크와 후보 시각화)
  발행 : /arms/detector_status  std_msgs/Float32MultiArray (검출·처리시간 진단값)

파라미터 (ros2 param set /arms_detection_node ...)
  use_hsv                      : bool  HSV 후보 제안 on/off, off=순수 전체화면 YOLO (기본 true)
  yolo.full_fallback_interval  : int   전체 화면 YOLO fallback 주기 (기본 1)
  proc_width            : int   검출 처리 가로 해상도, 0=원본 (기본 320)
  filter.confirm_frames : int   표적 확정에 필요한 연속 검출 수 (기본 3)
  filter.confirm_dist   : float 연속 판정 중심거리(정규화) (기본 0.035)
  filter.jump_gate      : float 확정 후 예측 대비 위치 급변 차단 거리, 0=off (기본 0.4)
  filter.max_age         : int  연속 miss 이만큼이면 표적 제거(LOST→REMOVED) (기본 8)
  roi.margin            : float /arms/roi_image 확대뷰 크롭 배율 (기본 1.8)
"""

import os
import time

import numpy as np
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray

from arms_msgs.msg import BoundingBox, DetectionArray


# ---------------------------------------------------------------------------
# Image conversion helpers (cv_bridge 없이)
# ---------------------------------------------------------------------------

_MORPH_KERNEL = np.ones((3, 3), np.uint8)


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
# 표적 필터 (KF 평활 + 이상치 게이트) — detection 노드 내부, control 에 투명
# ---------------------------------------------------------------------------

def _center_dist(a, b) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


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

    def coast(self):
        """측정 없이 예측만 한 스텝 진행(등속도 외삽). 다음 스텝이 이어지도록
        statePost 를 예측값으로 갱신하고 예측 위치를 반환한다."""
        if self._kf is None:
            return None
        pred = self._kf.predict()
        self._kf.statePost = pred.copy()
        self._kf.errorCovPost = self._kf.errorCovPre.copy()
        return float(pred[0, 0]), float(pred[1, 0])


class _TargetFilter:
    """매 프레임 전체 검출 결과를 칼만 필터로 평활하고, 확정된 표적에서 크게 튄
    검출(오인식)을 걸러 단일 표적을 낸다. 트래커(CSRT)는 쓰지 않는다.

    상태 (진단/UI 표시용 라벨):
      NEW      확정 전 — 연속 일치 카운트 중(스푸리어스 배제)
      TRACKED  확정 + 이번 프레임 검출 채택
      LOST     확정 + 순간 놓침 — KF 예측으로 외삽(coast). 검출 복귀 시 TRACKED
      REMOVED  놓침이 max_age 만큼 지속 → 표적 제거. 다음 검출은 NEW 로 시작
    전이:  NEW → TRACKED ⇄ LOST → REMOVED → (다음 검출) NEW
    control 엔 투명 — 더 연속적인 /arms/detections 를 낼 뿐이다."""
    NEW, TRACKED, LOST, REMOVED = "NEW", "TRACKED", "LOST", "REMOVED"

    def __init__(self):
        self._kalman = _CenterKalman()
        self._center = None      # 확정 표적의 최신 KF 중심 (게이트 기준)
        self._prev = None        # NEW 연속판정용 직전 중심
        self._streak = 0
        self._miss = 0
        self._last_box = None    # 마지막 발행 박스 (LOST 외삽 시 크기/클래스 재사용)
        self.state = self.NEW

    def reset(self):
        self._kalman.reset()
        self._center = None
        self._prev = None
        self._streak = 0
        self._miss = 0
        self._last_box = None
        self.state = self.NEW

    def update(self, det, cfg):
        """det: 이번 프레임 검출 박스(BoundingBox) 또는 None. 발행할 박스 반환(없으면 None)."""
        if self.state == self.REMOVED:
            self.reset()                 # 이전 프레임 제거됨 → 새 표적으로 다시 시작(NEW)
        if self.state == self.NEW:
            return self._new(det, cfg)
        return self._track(det, cfg)     # TRACKED / LOST

    def _new(self, det, cfg):
        """NEW: 같은 위치 연속검출(스푸리어스 1프레임 배제)로 표적을 확정하면 TRACKED."""
        if det is None:
            self._streak = 0
            self._prev = None
            self.state = self.NEW
            return None
        c = (det.x_center, det.y_center)
        if self._prev is not None and \
                _center_dist(c, self._prev) < cfg["confirm_dist"]:
            self._streak += 1
        else:
            self._streak = 1
        self._prev = c
        if self._streak >= cfg["confirm_frames"]:
            sx, sy = self._kalman.update(c[0], c[1])   # KF 초기화
            det.x_center = float(np.clip(sx, 0.0, 1.0))
            det.y_center = float(np.clip(sy, 0.0, 1.0))
            self._center = (sx, sy)
            self._last_box = det
            self._miss = 0
            self.state = self.TRACKED
        else:
            self.state = self.NEW
        return det   # 확정 전엔 실검출 그대로 발행

    def _track(self, det, cfg):
        """TRACKED/LOST: KF 예측 위치 기준 게이트 → 통과분만 평활 발행(TRACKED),
        미검출/게이트아웃은 KF 예측으로 외삽(LOST), max_age 만큼 지속되면 REMOVED."""
        gate = cfg["jump_gate"]
        pred = self._kalman.predicted_center() or self._center
        accept = det is not None and (
            gate <= 0.0 or
            _center_dist((det.x_center, det.y_center), pred) <= gate)
        if accept:
            sx, sy = self._kalman.update(det.x_center, det.y_center)
            self._center = (sx, sy)
            self._miss = 0
            self.state = self.TRACKED
            det.x_center = float(np.clip(sx, 0.0, 1.0))
            det.y_center = float(np.clip(sy, 0.0, 1.0))
            self._last_box = det
            return det
        # 미검출 또는 이상치(예측에서 크게 벗어남) → 표적을 놓침.
        self._miss += 1
        if self._miss >= cfg["max_age"]:
            self.state = self.REMOVED    # 오래 놓침 → 표적 제거(다음 프레임 NEW 로 리셋)
            self._last_box = None
            return None
        coasted = self._kalman.coast()   # 측정 없이 예측만 외삽
        if coasted is None:
            self.state = self.REMOVED
            return None
        self._center = coasted
        self.state = self.LOST
        return self._lost_box(coasted)

    def _lost_box(self, center):
        """LOST 외삽 발행 박스: 마지막 박스의 크기·클래스에 예측 중심만 얹는다."""
        if self._last_box is None:
            return None
        b = BoundingBox()
        b.x_center = float(np.clip(center[0], 0.0, 1.0))
        b.y_center = float(np.clip(center[1], 0.0, 1.0))
        b.width = self._last_box.width
        b.height = self._last_box.height
        b.confidence = self._last_box.confidence
        b.class_id = self._last_box.class_id
        b.class_name = self._last_box.class_name
        return b


# ---------------------------------------------------------------------------

class ArmsDetectionNode(Node):
    def __init__(self):
        super().__init__("arms_detection_node")

        # 파라미터 기본값(= 기본 운용 환경 실외). 환경별 튜닝은 config/detection.yaml
        # (및 detection.indoor.yaml override)로 덮는다 — compose 가 --params-file 로 주입.
        self.declare_parameter("proc_width", 320)             # 검출 처리 가로 해상도[px], 0=원본
        self.declare_parameter("use_hsv", True)               # HSV 후보 제안 on/off (off=순수 전체화면 YOLO)
        # HSV→ROI YOLO 가 못 찾은 프레임에 전체 화면 YOLO 를 도는 주기(1=매프레임, 0=끔).
        self.declare_parameter("yolo.full_fallback_interval", 1)
        self.declare_parameter("yolo.proposal_crop_px", 192)  # HSV 후보 검증용 crop 한 변[px]

        # HSV 붉은색 후보 제안 (단독 LOCK 금지 — 최종 승인은 YOLO)
        self.declare_parameter("hsv.max_candidates", 3)       # YOLO 검증할 상위 후보 수
        self.declare_parameter("hsv.texture_shortlist", 10)   # 질감 계산 적용 상위 후보 수
        self.declare_parameter("hsv.min_area_ratio", 0.0004)  # 면적비 하한(노이즈 컷). 상한 없음(근접 표적 유지)
        self.declare_parameter("hsv.hue_sigma", 15.0)         # 빨강 원형 Gaussian 폭
        self.declare_parameter("hsv.prob_threshold", 0.25)    # 붉은색 확률 이진화 임계
        self.declare_parameter("hsv.sat_center", 100.0)       # 채도 sigmoid 중심
        self.declare_parameter("hsv.sat_scale", 40.0)
        self.declare_parameter("hsv.val_min", 40.0)           # 명도 하한
        self.declare_parameter("hsv.val_max_center", 230.0)   # 과포화(조명) 감점 중심
        self.declare_parameter("hsv.val_scale", 40.0)
        self.declare_parameter("hsv.min_circularity", 0.30)   # 원형도/종횡비 하한
        self.declare_parameter("hsv.texture_scale", 120.0)    # 매끈함 점수 스케일

        # 표적 필터 (KF 평활 + 이상치 게이트)
        #   상태 전이: NEW → TRACKED ⇄ LOST → REMOVED → (다음 검출) NEW
        self.declare_parameter("filter.confirm_frames", 3)    # NEW→TRACKED: 같은 위치 연속검출 이만큼이면 확정
        self.declare_parameter("filter.confirm_dist", 0.035)  # NEW: "같은 위치" 연속 판정 중심거리(정규화)
        self.declare_parameter("filter.jump_gate", 0.4)       # TRACKED 유지 게이트: 예측서 이만큼 넘게 튀면 기각→miss (0=off)
        self.declare_parameter("filter.max_age", 8)           # LOST→REMOVED: 연속 miss 이만큼이면 표적 제거
        self.declare_parameter("roi.margin", 1.8)             # /arms/roi_image 확대뷰 크롭 배율

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self._filter = _TargetFilter()
        self._frame_i = 0   # 전체화면 YOLO fallback 주기 카운터
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
        model_path = os.environ.get("ARMS_MODEL", "")
        # 고정 batch TensorRT engine은 여러 ndarray 입력을 못 받아 후보별 순차 추론한다.
        # .pt 모델은 list 입력으로 한 번에 batch 추론한다.
        self._yolo_batch_enabled = not model_path.lower().endswith(".engine")
        if model_path:
            try:
                from ultralytics import YOLO
                self.get_logger().info(f"Loading YOLO model: {model_path}")
                self._yolo = YOLO(model_path, task="detect")
                # 시작 시 워밍업 predict 1회 — 엔진 로드·CUDA 컨텍스트·오토튜닝을 여기서
                #   끝낸다. (1) 첫 프레임 지연 스파이크 제거, (2) OOM/버전불일치면 여기서
                #   바로 드러남. 정적 엔진은 로드 때 메모리를 다 잡으므로 여기서 통과하면
                #   이후 프레임은 같은 버퍼 재사용 → 자기 추론으로 OOM 나지 않는다.
                dummy = np.zeros((self._yolo_roi_imgsz, self._yolo_roi_imgsz, 3), np.uint8)
                self._yolo.predict(dummy, imgsz=self._yolo_roi_imgsz, verbose=False)
                self.get_logger().info("YOLO loaded + warmed up (in-process).")
            except Exception as e:
                self._yolo = None   # 로드/워밍업 실패 → 검출 비활성(노드는 유지)
                self.get_logger().error(
                    f"YOLO 로드/워밍업 실패 — 검출 비활성 (GPU OOM/엔진 불일치 등): {e}")
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

        self.get_logger().info(
            "arms_detection_node ready [HSV proposals → batched ROI YOLO + full fallback, KF filter]")

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
            "filter_ms": 0.0, "total_ms": 0.0,
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

        # 매 프레임 전체 검출 → KF 평활 + 이상치 게이트로 단일 표적 발행.
        cfg = self._filter_cfg()
        det = self._detect_acquire(bgr)
        filter_started = time.perf_counter()
        target = self._filter.update(det, cfg)
        self._diag["filter_ms"] = (time.perf_counter() - filter_started) * 1000.0
        self._diag["total_ms"] = (time.perf_counter() - frame_started) * 1000.0
        self._publish_detector_status()

        if (self._diag_hsv_mask is not None and
                self.pub_hsv_debug.get_subscription_count() > 0):
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
        if self.pub_debug.get_subscription_count() > 0:
            self._publish_debug(bgr, header, target, self._filter.state)

    def _filter_cfg(self) -> dict:
        g = self.get_parameter
        return {
            "confirm_frames":   int(g("filter.confirm_frames").value),
            "confirm_dist":     float(g("filter.confirm_dist").value),
            "jump_gate":        float(g("filter.jump_gate").value),
            "max_age":          int(g("filter.max_age").value),
        }

    def _detect_acquire(self, bgr):
        """HSV 후보를 먼저 만들고 ROI들을 한 번의 YOLO batch로 검증한다."""
        self._diag["mode"] = 0.0
        use_yolo = self._yolo is not None   # 모델 로드됐을 때만 YOLO. HSV 단독 발행은 금지.
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
        if self.pub_detstatus.get_subscription_count() == 0:
            return
        diag = self._diag
        yolo_box = diag["yolo_box"]
        hsv_box = diag["hsv_box"]
        yolo_ran = bool(diag["yolo_ran"])
        use_hsv = bool(self.get_parameter("use_hsv").value)
        yv = (-2.0 if self._yolo is None else       # 모델 없음(DEAD)
              (-1.0 if not yolo_ran else            # 이 프레임 미실행
               (float(yolo_box.confidence) if yolo_box else 0.0)))
        hv = (-1.0 if not use_hsv else
              (float(hsv_box.confidence) if hsv_box else 0.0))
        msg = Float32MultiArray()
        state_code = {"NEW": 0.0, "TRACKED": 1.0, "LOST": 2.0, "REMOVED": 3.0}.get(
            self._filter.state, -1.0)
        # 0~3은 기존 UI와 호환. 4 이후는 실시간 성능 진단용이다.
        msg.data = [
            yv, hv, -1.0, float(diag["mode"]),
            float(diag["hsv_count"]), float(diag["yolo_inputs"]),
            float(diag["yolo_accepted"]), float(diag["frame_interval_ms"]),
            float(diag["resize_ms"]), float(diag["hsv_ms"]),
            float(diag["yolo_ms"]), float(diag["filter_ms"]),
            float(diag["total_ms"]), state_code, float(self._frame_i),
            1.0 if diag["full_frame"] else 0.0,
        ]
        self.pub_detstatus.publish(msg)

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

        로드·워밍업은 시작 시(__init__) 이미 검증됐다. 운영 중 predict 예외는 드물지만,
        노드를 죽이면 재시작→'CUDA device busy' 폭주가 나므로, 이 프레임만 miss 처리한다.
        """
        if not images:
            return []
        if self._yolo is None:
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
                self._diag["yolo_ms"] += (time.perf_counter() - yolo_started) * 1000.0
            self.get_logger().warning(
                f"YOLO 추론 실패 — 이 프레임 검출 보류: {e}", throttle_duration_sec=2.0)
            return [None] * len(images)
        if self._diag is not None:
            self._diag["yolo_ms"] += (
                time.perf_counter() - yolo_started) * 1000.0
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
        min_circ = float(self.get_parameter("hsv.min_circularity").value)
        preliminary = []
        for c in cnts:
            x, y, bw, bh = cv2.boundingRect(c)
            area_px = float(bw * bh)
            area_frac = area_px / float(w * h)
            # 하한(노이즈)만 컷한다. 상한은 없다 — 표적에 접근할수록 bbox 가 커지는 게
            # 정상(요격 성공 조건)이라, 최대 크기로 자르면 근접 시 추적이 끊긴다.
            if area_frac < min_frac:
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
            box.confidence = float(min(
                0.60, 0.50 * color_score + 0.40 * shape_score + 0.10 * smooth_score))
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

        # 중앙 마커 + 필터 상태 (TRACKED=초록, 그 외=주황)
        state_col = (0, 220, 0) if state == _TargetFilter.TRACKED else (0, 165, 255)
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
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C / docker stop(SIGINT·SIGTERM): rclpy 가 컨텍스트를 이미 닫으므로
        # 여기서 조용히 빠져나온다(아래 rclpy.ok() 가드로 이중 shutdown 방지).
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
