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

ROI 추적
  - 타겟 검출 후 중심 주변에 여유 있는 ROI 생성 → 다음 프레임은 ROI 내 검출 우선.
  - ROI 내 연속 미검출 roi_miss_limit 회 후 full scan 으로 복귀.
  - ROI 크롭 이미지는 /arms/roi_image 로 발행 (ROI 활성 시만).

토픽
  구독 : /arms/image_raw        (sensor_msgs/Image)
  구독 : /arms/yolo_detections  (arms_msgs/DetectionArray)  ← YOLO 도커가 발행
  발행 : /arms/detections       (arms_msgs/DetectionArray)  ← control 노드가 받음
  발행 : /arms/debug_image      (sensor_msgs/Image, bgr8)   ← bbox 시각화
  발행 : /arms/roi_image        (sensor_msgs/Image, bgr8)   ← ROI 크롭 (ROI 활성 시)

파라미터(ros2 param set /arms_detection_node ...)
  mode                : "both" | "yolo" | "cv"      (기본 both)
  cv.diff_thresh      : 대비 임계값 (기본 18)
  cv.bg_blur          : 배경 추정 블러 커널 (기본 21, 홀수)
  cv.min_area_ratio   : 최소 blob 면적비 (기본 0.00002 — 아주 작은 점도 허용)
  cv.max_area_ratio   : 최대 blob 면적비 (기본 0.05 — 너무 큰 건 배경/근접물 제외)
  publish_debug       : 디버그 영상 발행 (기본 True)
  roi_margin          : ROI 여백 배율 (기본 0.30 — bbox 크기의 30% 추가)
  roi_miss_limit      : 연속 미검출 후 full scan 복귀 횟수 (기본 3)

실행
  source /opt/ros/humble/setup.bash
  source <arms_ws>/install/setup.bash
  python3 fusion_detector.py
"""

#!/usr/bin/env python3
"""
fusion_detector.py — A.R.M.S. YOLO + CV(시계열 대비기반) 융합 검출기
구름 노이즈 및 가제보 튀는 현상 개선 버전
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
        super().__init__("arms_detection_node")

        # --- 파라미터 ---
        self.declare_parameter("use_hsv", True)        # 색(빨강) 검출
        self.declare_parameter("use_yolo", False)      # YOLO 도커 검출
        self.declare_parameter("use_absdiff", True)    # 대비(튀는 점) 검출
        self.declare_parameter("cv.diff_thresh", 20)   # 이진화 임계값 (구름 상태에 따라 15~25 조절)
        self.declare_parameter("cv.pre_blur", 3)       # 노이즈 제거 가우시안
        self.declare_parameter("cv.max_area_ratio", 0.02) # 구름 유입 방지를 위해 최대 크기 제한 축소
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("roi_margin", 0.30)     # ROI 여백
        self.declare_parameter("roi_miss_limit", 3)    # 연속 미검출 N회 후 full scan 복귀

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # 최신 YOLO 검출 캐시
        self._yolo_dets = []          # list[BoundingBox]

        # ROI 상태
        self._roi = None              # (x1, y1, x2, y2)
        self._roi_miss = 0            # 연속 미검출 카운트

        # 프레임 간 차이(Temporal Diff)를 위한 직전 프레임 캐시 정보
        # 중요: ROI 크롭 영역이 매번 바뀔 수 있으므로 full frame 전체를 백업하거나
        # 혹은 crop 영역에 맵핑되는 풀 그레이스케일 이미지를 유지해야 합니다.
        self._prev_gray_full = None

        self.sub_img = self.create_subscription(
            Image, "/arms/image_raw", self.cb_image, qos)
        self.sub_yolo = self.create_subscription(
            DetectionArray, "/arms/yolo_detections", self.cb_yolo, 10)

        self.pub_det = self.create_publisher(DetectionArray, "/arms/detections", 10)
        self.pub_dbg = self.create_publisher(Image, "/arms/debug_image", 10)
        self.pub_roi = self.create_publisher(Image, "/arms/roi_image", 10)

        self.get_logger().info("fusion_detector ready (Optimized for Clouds & Gazebo).")

    def cb_yolo(self, msg: DetectionArray):
        self._yolo_dets = list(msg.detections)

    def _make_roi(self, box: BoundingBox, img_w: int, img_h: int, margin: float):
        cx = box.x_center * img_w
        cy = box.y_center * img_h
        bw = max(box.width * img_w, 20)
        bh = max(box.height * img_h, 20)
        half_w = bw / 2 * (1.0 + margin)
        half_h = bh / 2 * (1.0 + margin)
        x1 = int(max(0,     cx - half_w))
        y1 = int(max(0,     cy - half_h))
        x2 = int(min(img_w, cx + half_w))
        y2 = int(min(img_h, cy + half_h))
        return (x1, y1, x2, y2)

    def _to_full_frame(self, box: BoundingBox, roi, img_w: int, img_h: int) -> BoundingBox:
        if roi is None:
            return box
        x1, y1, x2, y2 = roi
        rw, rh = x2 - x1, y2 - y1
        b = BoundingBox()
        b.x_center   = (x1 + box.x_center * rw) / img_w
        b.y_center   = (y1 + box.y_center * rh) / img_h
        b.width      = box.width  * rw / img_w
        b.height     = box.height * rh / img_h
        b.confidence = box.confidence
        b.class_id   = box.class_id
        b.class_name = box.class_name
        return b

    def detect_hsv(self, bgr: np.ndarray):
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, (0, 90, 60), (10, 255, 255))
        m2 = cv2.inRange(hsv, (170, 90, 60), (180, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 4:
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
    # CV: 개선된 대비/움직임 기반 점 검출
    # -----------------------------------------------------------------
    def detect_cv(self, bgr_crop: np.ndarray, gray_crop: np.ndarray, prev_gray_crop: np.ndarray):
        h, w = bgr_crop.shape[:2]
        diff_thresh = int(self.get_parameter("cv.diff_thresh").value)
        max_area = float(self.get_parameter("cv.max_area_ratio").value) * w * h

        if prev_gray_crop is None:
            return None

        # 1. 프레임 간의 absdiff 계산 (움직이지 않는 구름 억제)
        diff = cv2.absdiff(gray_crop, prev_gray_crop)
        _, mask = cv2.threshold(diff, diff_thresh, 255, cv2.THRESH_BINARY)

        # 2. 모폴로지 열기(Opening) 연산으로 미세 구름 노이즈 파편 지우기
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = -1.0
        
        for c in cnts:
            x, y, bw, bh = cv2.boundingRect(c)
            a_px = bw * bh
            
            # [개선 조건 1] 너무 미세한 연무 노이즈 차단 및 최대 크기 차단
            if a_px < 4 or a_px > max_area:
                continue
                
            # [개선 조건 2] 가로세로 비율 필터링 (구름 찌꺼기는 보통 한쪽으로 길쭉함)
            # 원거리 점 타겟은 1.0(정사각형)에 가까워야 함
            aspect_ratio = float(bw) / float(bh)
            if aspect_ratio < 0.25 or aspect_ratio > 4.0:
                continue

            roi_diff = diff[max(0, y):y + bh, max(0, x):x + bw]
            if roi_diff.size == 0:
                continue
                
            contrast = float(roi_diff.max())
            # 형태 안정성이 높고 대비가 뚜렷할수록 고득점
            score = contrast * (a_px ** 0.2) 
            
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
        margin = float(self.get_parameter("roi_margin").value)
        miss_limit = int(self.get_parameter("roi_miss_limit").value)

        want_hsv     = bool(self.get_parameter("use_hsv").value)
        want_yolo    = bool(self.get_parameter("use_yolo").value)
        want_absdiff = bool(self.get_parameter("use_absdiff").value)

        # 공통 그레이스케일 전처리 (노이즈 선제거)
        pre = int(self.get_parameter("cv.pre_blur").value)
        if pre % 2 == 0: pre += 1
        gray_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray_full = cv2.GaussianBlur(gray_full, (pre, pre), 0)

        # ROI 크롭 범위 확정
        roi = self._roi
        if roi is not None:
            rx1, ry1, rx2, ry2 = roi
            crop = bgr[ry1:ry2, rx1:rx2]
            if crop.size == 0:
                roi = None
                crop = bgr
                gray_crop = gray_full
                prev_gray_crop = self._prev_gray_full if self._prev_gray_full is not None else None
            else:
                gray_crop = gray_full[ry1:ry2, rx1:rx2]
                prev_gray_crop = self._prev_gray_full[ry1:ry2, rx1:rx2] if self._prev_gray_full is not None else None
        else:
            crop = bgr
            gray_crop = gray_full
            prev_gray_crop = self._prev_gray_full if self._prev_gray_full is not None else None

        # --- 검출 진행 ---
        yolo_box = None
        if want_yolo and self._yolo_dets:
            yolo_box = max(self._yolo_dets, key=lambda b: b.confidence)

        hsv_box_crop = self.detect_hsv(crop) if want_hsv else None
        
        # 수정된 시계열 absdiff 검출 함수 호출
        cv_box_crop  = self.detect_cv(crop, gray_crop, prev_gray_crop) if want_absdiff else None

        # 차후 프레임을 위해 풀 프레임 그레이 이미지 캐시 갱신
        self._prev_gray_full = gray_full.copy()

        # 상대 좌표 → 풀 프레임 좌표 복원
        hsv_box = self._to_full_frame(hsv_box_crop, roi, w, h) if hsv_box_crop else None
        cv_box  = self._to_full_frame(cv_box_crop,  roi, w, h) if cv_box_crop  else None

        # --- 데이터 융합(Fusion) ---
        cands = [b for b in (yolo_box, hsv_box, cv_box) if b is not None]
        if not cands:
            target = None
        elif len(cands) == 1:
            target = cands[0]
        else:
            target = self._fuse_all(cands)

        # --- ROI 피드백 루프 상태 관리 ---
        if target is not None:
            self._roi = self._make_roi(target, w, h, margin)
            self._roi_miss = 0
        elif roi is not None:
            self._roi_miss += 1
            if self._roi_miss >= miss_limit:
                self._roi = None
                self._roi_miss = 0
        else:
            self._roi_miss = 0

        # 결과 토픽 발행
        out = DetectionArray()
        out.header = msg.header
        if target is not None:
            out.detections.append(target)
        self.pub_det.publish(out)

        # ROI 이미지 채널 발행
        if roi is not None and self.pub_roi.get_subscription_count() > 0:
            self.pub_roi.publish(bgr_to_imgmsg(crop, msg.header))

        # 디버그 렌더링
        if bool(self.get_parameter("publish_debug").value):
            label = "+".join(
                n for n, on in (("HSV", want_hsv), ("YOLO", want_yolo), ("ABSDIFF", want_absdiff)) if on
            ) or "OFF"
            self._publish_debug(bgr, msg.header, yolo_box, hsv_box, cv_box, target, label, roi)

    # -----------------------------------------------------------------
    def _fuse_all(self, boxes):
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
    def _publish_debug(self, bgr, header, yolo_box, hsv_box, cv_box, target, mode, roi):
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
            if x2 - x1 < 6:
                x1, x2 = int(cx - 8), int(cx + 8)
            if y2 - y1 < 6:
                y1, y2 = int(cy - 8), int(cy + 8)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, label, (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        draw(yolo_box, (0, 200, 0), "YOLO")
        draw(hsv_box, (200, 0, 200), "HSV")
        draw(cv_box, (0, 220, 220), "ABSDIFF")
        
        if target is not None:
            tx, ty = int(target.x_center * w), int(target.y_center * h)
            cv2.drawMarker(img, (tx, ty), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
            cv2.putText(img, "TARGET", (tx + 10, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        if roi is not None:
            rx1, ry1, rx2, ry2 = roi
            cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (255, 100, 0), 1)
            cv2.putText(img, "ROI", (rx1, max(12, ry1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 0), 1, cv2.LINE_AA)

        cv2.drawMarker(img, (w // 2, h // 2), (180, 180, 180), cv2.MARKER_TILTED_CROSS, 16, 1)
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