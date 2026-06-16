#!/usr/bin/env python3
"""
fusion_detector.py — A.R.M.S. YOLO + CV(대비기반) 융합 검출기

목적
  - 가까운 표적: YOLO 가 형태로 잡음
  - 멀어서 점이 된 표적: CV(대비/absdiff) 가 색·형태 없이 잡음
  → 둘을 합쳐서 원거리/근거리 모두 추적 유지.

핵심
  - CV 는 "특정 색"을 쓰지 않는다. 하늘(균일 배경)에서 주변과 밝기가
    다른(밝든 어둡든) 작은 점을 absdiff 로 검출 → 실전 일반화에 유리.
  - 모드 토글(ros2 param `mode`: yolo / cv / both) — 패널 버튼으로 전환.
  - 둘 다 잡히면 두 중심의 평균을 target 으로, 하나만 잡혀도 작동,
    둘 다 없으면 빈 DetectionArray(타겟 상실).

토픽
  구독 : /arms/image_raw        (sensor_msgs/Image)
  구독 : /arms/yolo_detections  (arms_msgs/DetectionArray)  ← YOLO 도커가 발행
  발행 : /arms/detections       (arms_msgs/DetectionArray)  ← control 노드가 받음
  발행 : /arms/debug_image      (sensor_msgs/Image, bgr8)   ← bbox 시각화

파라미터(ros2 param set /fusion_detector ...)
  mode                : "both" | "yolo" | "cv"      (기본 both)
  cv.diff_thresh      : 대비 임계값 (기본 18)
  cv.bg_blur          : 배경 추정 블러 커널 (기본 21, 홀수)
  cv.min_area_ratio   : 최소 blob 면적비 (기본 0.00002 — 아주 작은 점도 허용)
  cv.max_area_ratio   : 최대 blob 면적비 (기본 0.05 — 너무 큰 건 배경/근접물 제외)
  publish_debug       : 디버그 영상 발행 (기본 True)

실행
  source /opt/ros/humble/setup.bash
  source <arms_ws>/install/setup.bash
  python3 fusion_detector.py
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from arms_msgs.msg import BoundingBox, DetectionArray


# ---------------------------------------------------------------------------
# 이미지 변환 (cv_bridge 없이)
# ---------------------------------------------------------------------------
def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    ch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding, 3)
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]  # RGB -> BGR
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


class FusionDetector(Node):
    def __init__(self):
        super().__init__("fusion_detector")

        # --- 파라미터 ---
        # 검출기 3개 독립 on/off (여러 개 켜면 다 적용 → 융합). 패널 버튼이 토글.
        self.declare_parameter("use_hsv", True)        # 색(빨강) 검출
        self.declare_parameter("use_yolo", False)      # YOLO 도커 검출
        self.declare_parameter("use_absdiff", True)    # 대비(튀는 점) 검출
        self.declare_parameter("cv.diff_thresh", 25)       # 노이즈보다 높게(점 peak는 충분히 큼)
        self.declare_parameter("cv.bg_blur", 15)           # 배경(하늘) 추정 median 커널
        self.declare_parameter("cv.pre_blur", 3)           # 노이즈 제거 가우시안
        self.declare_parameter("cv.max_area_ratio", 0.05)  # 너무 큰 건 배경/근접물 제외
        self.declare_parameter("publish_debug", True)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # 최신 YOLO 검출 캐시 (이미지 콜백에서 합침)
        self._yolo_dets = []          # list[BoundingBox]

        self.sub_img = self.create_subscription(
            Image, "/arms/image_raw", self.cb_image, qos)
        self.sub_yolo = self.create_subscription(
            DetectionArray, "/arms/yolo_detections", self.cb_yolo, 10)

        self.pub_det = self.create_publisher(DetectionArray, "/arms/detections", 10)
        self.pub_dbg = self.create_publisher(Image, "/arms/debug_image", 10)

        self.get_logger().info("fusion_detector ready (YOLO + contrast-CV, no color).")

    # -- YOLO 결과 캐시 --
    def cb_yolo(self, msg: DetectionArray):
        self._yolo_dets = list(msg.detections)

    # -----------------------------------------------------------------
    # CV: HSV 색 검출 (빨간 풍선 색으로 찾기 — 옛 redball 방식)
    #   하늘 배경에서 빨간색 영역을 색상으로 분리. 색이 분명할 때 강력.
    #   반환: BoundingBox 1개 또는 None
    # -----------------------------------------------------------------
    def detect_hsv(self, bgr: np.ndarray):
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # 빨강은 H 양끝(0 근처 + 180 근처) 둘 다
        m1 = cv2.inRange(hsv, (0, 90, 60), (10, 255, 255))
        m2 = cv2.inRange(hsv, (170, 90, 60), (180, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 4:                       # 너무 작은 노이즈 제외
            return None
        x, y, bw, bh = cv2.boundingRect(c)
        box = BoundingBox()
        box.x_center = float((x + bw / 2) / w)
        box.y_center = float((y + bh / 2) / h)
        box.width = float(bw / w)
        box.height = float(bh / h)
        box.confidence = float(min(0.99, 0.7 + area / (w * h) * 20.0))
        box.class_id = 2
        box.class_name = "hsv_red"
        return box

    # -----------------------------------------------------------------
    # CV: 대비 기반 점 검출 (색·형태 무관)
    #   - 하늘(균일 배경)을 큰 블러로 추정 → absdiff 로 "튀는 점" 검출
    #   - 밝은 점/어두운 점 모두 잡힘
    #   반환: BoundingBox 1개(가장 그럴듯한 점) 또는 None
    # -----------------------------------------------------------------
    def detect_cv(self, bgr: np.ndarray):
        h, w = bgr.shape[:2]
        diff_thresh = int(self.get_parameter("cv.diff_thresh").value)
        bg_k = int(self.get_parameter("cv.bg_blur").value)
        pre = int(self.get_parameter("cv.pre_blur").value)
        if bg_k % 2 == 0:
            bg_k += 1
        if pre % 2 == 0:
            pre += 1
        max_area = float(self.get_parameter("cv.max_area_ratio").value) * w * h

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (pre, pre), 0)      # 노이즈 먼저 제거
        bg = cv2.medianBlur(gray, bg_k)                   # 배경(하늘) 추정
        diff = cv2.absdiff(gray, bg)                      # 밝은/어두운 점 모두 양수
        _, mask = cv2.threshold(diff, diff_thresh, 255, cv2.THRESH_BINARY)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = -1.0
        for c in cnts:
            x, y, bw, bh = cv2.boundingRect(c)
            a_px = bw * bh
            if a_px > max_area:                            # 너무 크면 배경/근접물 제외
                continue
            roi = diff[max(0, y):y + bh, max(0, x):x + bw]
            if roi.size == 0:
                continue
            contrast = float(roi.max())                    # peak 대비 (작은 점에 유리)
            score = contrast * (a_px ** 0.3)               # 대비 강한 점 우선
            if score > best_score:
                best_score = score
                best = (x, y, bw, bh, contrast)

        if best is None:
            return None
        x, y, bw, bh, contrast = best
        box = BoundingBox()
        box.x_center = float((x + bw / 2) / w)
        box.y_center = float((y + bh / 2) / h)
        box.width = float(bw / w)
        box.height = float(bh / h)
        box.confidence = float(min(0.99, 0.65 + contrast / 255.0))
        box.class_id = 1
        box.class_name = "cv_spot"
        return box

    # -----------------------------------------------------------------
    def cb_image(self, msg: Image):
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return

        h, w = bgr.shape[:2]
        # 검출기 3개 독립 토글 (여러 개 켜져 있으면 전부 돌려서 융합)
        want_hsv     = bool(self.get_parameter("use_hsv").value)
        want_yolo    = bool(self.get_parameter("use_yolo").value)
        want_absdiff = bool(self.get_parameter("use_absdiff").value)

        # --- 각 검출기 결과 ---
        yolo_box = None
        if want_yolo and self._yolo_dets:
            yolo_box = max(self._yolo_dets, key=lambda b: b.confidence)

        hsv_box = self.detect_hsv(bgr) if want_hsv else None
        cv_box = self.detect_cv(bgr) if want_absdiff else None

        # --- target 결정: 켜진 검출기 중 잡힌 것들을 융합(중심 평균, 최고신뢰 박스) ---
        cands = [b for b in (yolo_box, hsv_box, cv_box) if b is not None]
        if not cands:
            target = None
        elif len(cands) == 1:
            target = cands[0]
        else:
            target = self._fuse_all(cands)

        out = DetectionArray()
        out.header = msg.header
        if target is not None:
            out.detections.append(target)
        self.pub_det.publish(out)

        # --- 디버그 영상 ---
        if bool(self.get_parameter("publish_debug").value):
            label = "+".join(
                n for n, on in (("HSV", want_hsv), ("YOLO", want_yolo), ("ABSDIFF", want_absdiff)) if on
            ) or "OFF"
            self._publish_debug(bgr, msg.header, yolo_box, hsv_box, cv_box, target, label)

    # -----------------------------------------------------------------
    def _fuse_all(self, boxes):
        """켜진 검출기들이 동시에 잡으면 융합: 중심은 평균, 크기/신뢰는 최고신뢰 박스."""
        best = max(boxes, key=lambda b: b.confidence)
        t = BoundingBox()
        t.x_center = sum(b.x_center for b in boxes) / len(boxes)
        t.y_center = sum(b.y_center for b in boxes) / len(boxes)
        t.width = best.width
        t.height = best.height
        t.confidence = best.confidence
        t.class_id = 9
        t.class_name = "fused"
        return t

    # -----------------------------------------------------------------
    def _publish_debug(self, bgr, header, yolo_box, hsv_box, cv_box, target, mode):
        img = bgr.copy()
        h, w = img.shape[:2]

        def draw(box, color, label):
            if box is None:
                return
            bw = box.width * w
            bh = box.height * h
            cx = box.x_center * w
            cy = box.y_center * h
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            # 점이 너무 작으면 최소 박스 크기 보장(보이게)
            if x2 - x1 < 6:
                x1, x2 = int(cx - 8), int(cx + 8)
            if y2 - y1 < 6:
                y1, y2 = int(cy - 8), int(cy + 8)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, label, (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        # YOLO=초록, HSV=자홍, CV=노랑, target=빨강 십자
        draw(yolo_box, (0, 200, 0), "YOLO")
        draw(hsv_box, (200, 0, 200), "HSV")
        draw(cv_box, (0, 220, 220), "ABSDIFF")
        if target is not None:
            tx, ty = int(target.x_center * w), int(target.y_center * h)
            cv2.drawMarker(img, (tx, ty), (0, 0, 255),
                           cv2.MARKER_CROSS, 22, 2)
            cv2.putText(img, "TARGET", (tx + 10, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        # 화면 중앙 십자(조준 기준)
        cv2.drawMarker(img, (w // 2, h // 2), (180, 180, 180),
                       cv2.MARKER_TILTED_CROSS, 16, 1)
        # 모드 표시
        cv2.putText(img, f"MODE: {mode.upper()}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        self.pub_dbg.publish(bgr_to_imgmsg(img, header))


def main():
    rclpy.init()
    node = FusionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
