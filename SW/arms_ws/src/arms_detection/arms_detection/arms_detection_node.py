#!/usr/bin/env python3
"""
arms_detection_node — A.R.M.S. 다중 검출기 (우선순위 기반)

검출기 우선순위: YOLO > HSV > ABSDIFF
  - YOLO    : 외부 도커가 발행한 /arms/yolo_detections 캐시 사용
  - HSV     : 빨간색 영역 색상 분리 (근거리, 색이 선명할 때 강력)
  - ABSDIFF : 배경 대비 튀는 점 검출 (색·형태 무관, 원거리 소형 표적에 유리)

토픽
  구독 : /arms/image_raw        sensor_msgs/Image
  구독 : /arms/yolo_detections  arms_msgs/DetectionArray
  발행 : /arms/detections       arms_msgs/DetectionArray
  발행 : /arms/debug_image      sensor_msgs/Image  (bgr8, 시각화)
  발행 : /arms/debug_absdiff    sensor_msgs/Image  (mono8, absdiff 이진 마스크)

파라미터 (ros2 param set /arms_detection_node ...)
  use_yolo              : bool  (기본 true)
  use_hsv               : bool  (기본 true)
  use_absdiff           : bool  (기본 true)
  yolo_ttl              : float YOLO 결과 신선도 초, 초과 시 폴백 (기본 0.5)
  proc_width            : int   검출 처리 가로 해상도, 0=원본 (기본 480)
  absdiff.diff_thresh   : int   배경 대비 임계값 (기본 25)
  absdiff.bg_blur       : int   배경 추정 median 커널, 홀수 (기본 15)
  absdiff.pre_blur      : int   노이즈 제거 Gaussian 커널, 홀수 (기본 3)
  absdiff.max_area_ratio: float blob 최대 면적비 (기본 0.05)
  publish_debug         : bool  디버그 영상 발행 (기본 true)
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from arms_msgs.msg import BoundingBox, DetectionArray


# ---------------------------------------------------------------------------
# Image conversion helpers (cv_bridge 없이)
# ---------------------------------------------------------------------------

_OPEN_KERNEL = np.ones((3, 3), np.uint8)   # absdiff 마스크 노이즈 제거용


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

class ArmsDetectionNode(Node):
    def __init__(self):
        super().__init__("arms_detection_node")

        self.declare_parameter("use_yolo",   True)
        self.declare_parameter("use_hsv",    True)
        self.declare_parameter("use_absdiff", True)
        # YOLO 결과 신선도(초). 마지막 YOLO 수신 후 이 시간이 지나면 캐시를 무시하고
        # HSV/ABSDIFF 로 폴백 → 컨테이너가 죽어도 오래된 박스를 계속 쓰지 않는다.
        # 비동기 YOLO(저속) 와 30fps 루프의 레이트 브리징을 위한 짧은 hold.
        self.declare_parameter("yolo_ttl", 0.5)
        # 검출 처리 해상도(가로 px). 0=원본. 출력은 비율이라 정확도 무관, CV 비용만 절감.
        self.declare_parameter("proc_width", 480)
        self.declare_parameter("absdiff.diff_thresh",    25)
        self.declare_parameter("absdiff.bg_blur",        15)
        self.declare_parameter("absdiff.pre_blur",        3)
        self.declare_parameter("absdiff.max_area_ratio", 0.05)
        self.declare_parameter("publish_debug", True)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self._yolo_cache: list[BoundingBox] = []
        self._yolo_stamp_ns: int = 0   # 마지막 YOLO 수신 시각 (TTL 판정용)

        self.create_subscription(Image, "/arms/image_raw", self._cb_image, qos)
        self.create_subscription(DetectionArray, "/arms/yolo_detections", self._cb_yolo, 10)

        self.pub_det     = self.create_publisher(DetectionArray, "/arms/detections",    10)
        self.pub_debug   = self.create_publisher(Image,          "/arms/debug_image",   10)
        self.pub_absdiff = self.create_publisher(Image,          "/arms/debug_absdiff", 10)

        self.get_logger().info("arms_detection_node ready  [priority: YOLO > HSV > ABSDIFF]")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _cb_yolo(self, msg: DetectionArray):
        self._yolo_cache = list(msg.detections)
        self._yolo_stamp_ns = self.get_clock().now().nanoseconds

    def _cb_image(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
            return   # 빈 프레임(image_publisher 루프 경계 등) → 스킵
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return
        if bgr is None or bgr.size == 0:
            return

        # 검출 처리 다운스케일 (출력은 비율이라 정확도 무관, CV 비용만 절감)
        proc_w = int(self.get_parameter("proc_width").value)
        if proc_w > 0 and bgr.shape[1] > proc_w:
            ph = int(bgr.shape[0] * proc_w / bgr.shape[1])
            bgr = cv2.resize(bgr, (proc_w, ph), interpolation=cv2.INTER_AREA)

        use_yolo    = bool(self.get_parameter("use_yolo").value)
        use_hsv     = bool(self.get_parameter("use_hsv").value)
        use_absdiff = bool(self.get_parameter("use_absdiff").value)

        yolo_box    = self._detect_yolo()             if use_yolo    else None
        hsv_box     = self._detect_hsv(bgr)           if use_hsv     else None
        absdiff_box = self._detect_absdiff(bgr, msg.header) if use_absdiff else None

        # 우선순위: YOLO > HSV > ABSDIFF
        target = yolo_box or hsv_box or absdiff_box

        out = DetectionArray()
        out.header = msg.header
        if target is not None:
            out.detections.append(target)
        self.pub_det.publish(out)

        # 디버그 이미지는 구독자가 있을 때만 그려서 발행 (없으면 발행 비용 전부 절감)
        if bool(self.get_parameter("publish_debug").value) \
                and self.pub_debug.get_subscription_count() > 0:
            self._publish_debug(bgr, msg.header, yolo_box, hsv_box, absdiff_box, target,
                                use_yolo, use_hsv, use_absdiff)

    # ------------------------------------------------------------------
    # Detectors
    # ------------------------------------------------------------------

    def _detect_yolo(self) -> BoundingBox | None:
        if not self._yolo_cache:
            return None
        ttl = float(self.get_parameter("yolo_ttl").value)
        age = (self.get_clock().now().nanoseconds - self._yolo_stamp_ns) / 1e9
        if age > ttl:
            return None   # 오래된 YOLO 결과 → 폴백 (컨테이너 미기동/지연)
        return max(self._yolo_cache, key=lambda b: b.confidence)

    def _detect_hsv(self, bgr: np.ndarray) -> BoundingBox | None:
        """빨간 영역 HSV 색상 분리."""
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, (  0, 90, 60), ( 10, 255, 255)),
            cv2.inRange(hsv, (170, 90, 60), (180, 255, 255)),
        )
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 4:
            return None
        x, y, bw, bh = cv2.boundingRect(c)
        box = BoundingBox()
        box.x_center  = float((x + bw / 2) / w)
        box.y_center  = float((y + bh / 2) / h)
        box.width     = float(bw / w)
        box.height    = float(bh / h)
        box.confidence = float(min(0.99, 0.7 + area / (w * h) * 20.0))
        box.class_id  = 2
        box.class_name = "hsv_red"
        return box

    def _detect_absdiff(self, bgr: np.ndarray, header=None) -> BoundingBox | None:
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
        # 잔점(엣지 노이즈) 제거 → contour 수 급감 → 아래 루프 대폭 가속 + 노이즈 정리
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _OPEN_KERNEL)

        if header is not None and self.pub_absdiff.get_subscription_count() > 0:
            self.pub_absdiff.publish(mono_to_imgmsg(mask, header))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_score = None, -1.0
        for c in cnts:
            x, y, bw, bh = cv2.boundingRect(c)
            a_px = bw * bh
            if a_px > max_area:
                continue
            patch = diff[max(0, y):y + bh, max(0, x):x + bw]
            if patch.size == 0:
                continue
            contrast = float(patch.max())
            score = contrast * (a_px ** 0.3)
            if score > best_score:
                best_score = score
                best = (x, y, bw, bh, contrast)

        if best is None:
            return None
        x, y, bw, bh, contrast = best
        box = BoundingBox()
        box.x_center  = float((x + bw / 2) / w)
        box.y_center  = float((y + bh / 2) / h)
        box.width     = float(bw / w)
        box.height    = float(bh / h)
        box.confidence = float(min(0.99, 0.65 + contrast / 255.0))
        box.class_id  = 1
        box.class_name = "absdiff_spot"
        return box

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------

    def _publish_debug(self, bgr, header, yolo_box, hsv_box, absdiff_box, target,
                       use_yolo, use_hsv, use_absdiff):
        img = bgr.copy()
        h, w = img.shape[:2]

        def draw_box(box, color, label):
            if box is None:
                return
            cx, cy = box.x_center * w, box.y_center * h
            bw, bh = box.width * w, box.height * h
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            if x2 - x1 < 6:
                x1, x2 = int(cx - 8), int(cx + 8)
            if y2 - y1 < 6:
                y1, y2 = int(cy - 8), int(cy + 8)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, label, (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        draw_box(yolo_box,    (0, 200,   0), "YOLO")
        draw_box(hsv_box,     (200, 0, 200), "HSV")
        draw_box(absdiff_box, (0, 220, 220), "ABSDIFF")

        if target is not None:
            tx, ty = int(target.x_center * w), int(target.y_center * h)
            cv2.drawMarker(img, (tx, ty), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
            cv2.putText(img, "TARGET", (tx + 10, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.drawMarker(img, (w // 2, h // 2), (180, 180, 180),
                       cv2.MARKER_TILTED_CROSS, 16, 1)

        active = "+".join(n for n, on in
                          (("YOLO", use_yolo), ("HSV", use_hsv), ("ABSDIFF", use_absdiff)) if on) or "OFF"
        cv2.putText(img, active, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

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
