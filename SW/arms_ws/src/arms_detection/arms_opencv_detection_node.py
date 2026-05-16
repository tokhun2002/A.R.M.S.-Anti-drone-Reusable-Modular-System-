#!/usr/bin/env python3
"""
OpenCV 원 감지 기반 Detection Node (SITL / 개발용)

Subscribe : /arms/image_raw  (sensor_msgs/Image)
Publish   : /arms/detections (arms_msgs/DetectionArray)

HoughCircles로 원형 물체(풍선)를 감지해 YOLO 노드와 동일한 인터페이스로 퍼블리시.
YOLO 없이 빠르게 동작 검증할 때 사용.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from arms_msgs.msg import BoundingBox, DetectionArray

# ── 파라미터 (런타임에 ros param으로 override 가능) ───────────────────
DEFAULTS = {
    "blur_ksize":    11,    # GaussianBlur 커널 크기 (홀수)
    "dp":            1.2,   # HoughCircles 누산기 해상도 비율
    "min_dist":      50,    # 원 중심 간 최소 거리 (px)
    "param1":        80,    # Canny edge 상위 임계값
    "param2":        30,    # 누산기 임계값 (낮을수록 민감)
    "min_radius":    10,    # 최소 반지름 (px)
    "max_radius":    200,   # 최대 반지름 (px)
    "confidence":    0.8,   # 퍼블리시할 고정 confidence 값
}


def imgmsg_to_numpy(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → HxWx3 uint8 BGR (cv_bridge 없이)"""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    bpp  = msg.step // msg.width
    enc  = msg.encoding.lower()

    if enc == "rgb8":
        return data.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
    elif enc == "bgr8":
        return data.reshape(msg.height, msg.width, 3)
    elif enc == "mono8":
        gray = data.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif enc in ("yuyv", "yuv422", "yuv422_yuy2"):
        return cv2.cvtColor(data.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_YUYV)
    else:
        img = data.reshape(msg.height, msg.width, bpp)
        if bpp == 2:
            return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
        elif bpp == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img


class ArmsOpencvDetectionNode(Node):
    def __init__(self):
        super().__init__("arms_opencv_detection_node")

        # ROS 파라미터 선언
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            Image, "/arms/image_raw", self.cb_image, best_effort_qos
        )
        self.pub = self.create_publisher(DetectionArray, "/arms/detections", 10)
        self.get_logger().info("arms_opencv_detection_node ready.")

    def cb_image(self, msg: Image):
        try:
            frame = imgmsg_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        h, w = frame.shape[:2]

        # 파라미터 읽기
        p = lambda k: self.get_parameter(k).value

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (p("blur_ksize"), p("blur_ksize")), 0)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=p("dp"),
            minDist=p("min_dist"),
            param1=p("param1"),
            param2=p("param2"),
            minRadius=p("min_radius"),
            maxRadius=p("max_radius"),
        )

        det_msg = DetectionArray()
        det_msg.header = msg.header

        if circles is not None:
            for cx, cy, r in np.round(circles[0]).astype(int):
                # 정규화 좌표 (YOLO 노드와 동일한 포맷)
                bbox = BoundingBox()
                bbox.x_center   = float(cx) / w
                bbox.y_center   = float(cy) / h
                bbox.width      = float(r * 2) / w
                bbox.height     = float(r * 2) / h
                bbox.confidence = float(p("confidence"))
                bbox.class_id   = 0
                bbox.class_name = "balloon"
                det_msg.detections.append(bbox)

        self.pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmsOpencvDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
