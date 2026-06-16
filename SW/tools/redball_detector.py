#!/usr/bin/env python3
"""
redball_detector.py  — SITL 전용 빨간 공(풍선) 검출기

목적:
  Docker/YOLO(torch/CUDA) 없이 OpenCV HSV 색 분할만으로 /arms/image_raw 에서
  빨간 구체를 찾아 /arms/detections 로 발행한다. SITL 파이프라인을
  end-to-end 로 굴리기 위한 "임시 perception" 노드.
  (실기체에서는 기존 arms_detection 컨테이너의 YOLO 를 그대로 쓰면 됨.)

구독 : /arms/image_raw   (sensor_msgs/Image, rgb8 가정)
발행 : /arms/detections  (arms_msgs/DetectionArray)

실행:
  source /opt/ros/humble/setup.bash
  source <arms_ws>/install/setup.bash
  python3 redball_detector.py
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from arms_msgs.msg import BoundingBox, DetectionArray

# 검출 파라미터 (필요시 조정)
MIN_AREA_RATIO = 0.0002   # 화면 면적 대비 최소 blob 크기 (노이즈 컷)
PUB_CONF = 0.90           # 검출 성공 시 confidence (SM threshold 0.65 통과용)


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    ch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding, 3)
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]  # RGB -> BGR for cv2
    return np.ascontiguousarray(img)


class RedBallDetector(Node):
    def __init__(self):
        super().__init__("redball_detector")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(Image, "/arms/image_raw", self.cb, qos)
        self.pub = self.create_publisher(DetectionArray, "/arms/detections", 10)
        self.get_logger().info("redball_detector ready (OpenCV HSV, no GPU).")

    def cb(self, msg: Image):
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return

        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # 빨강은 H 양 끝(0 근처 + 180 근처) 둘 다 잡는다
        m1 = cv2.inRange(hsv, (0, 90, 60), (10, 255, 255))
        m2 = cv2.inRange(hsv, (170, 90, 60), (180, 255, 255))
        mask = cv2.morphologyEx(m1 | m2, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))

        det = DetectionArray()
        det.header = msg.header

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area >= MIN_AREA_RATIO * w * h:
                x, y, bw, bh = cv2.boundingRect(c)
                box = BoundingBox()
                box.x_center = float((x + bw / 2) / w)
                box.y_center = float((y + bh / 2) / h)
                box.width = float(bw / w)
                box.height = float(bh / h)
                box.confidence = PUB_CONF
                box.class_id = 0
                box.class_name = "balloon"
                det.detections.append(box)

        self.pub.publish(det)


def main():
    rclpy.init()
    node = RedBallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
