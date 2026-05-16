"""
A.R.M.S. Detection Node — runs inside the Docker container.

Subscribes : /arms/image_raw  (sensor_msgs/Image)
Publishes  : /arms/detections (arms_msgs/DetectionArray)

Model path is read from the env var ARMS_MODEL (default: /models/drone.pt).
"""

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

from arms_msgs.msg import BoundingBox, DetectionArray

MODEL_PATH = os.environ.get("ARMS_MODEL", "/models/best.onnx")
CONFIDENCE = float(os.environ.get("ARMS_CONF", "0.5"))
IOU = float(os.environ.get("ARMS_IOU",  "0.45"))


def imgmsg_to_numpy(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → HxWx3 uint8 RGB (cv_bridge 없이)
    msg.step 기반으로 바이트 수를 자동 계산하므로 해상도/인코딩 무관하게 동작.
    """
    data = np.frombuffer(msg.data, dtype=np.uint8)
    bpp  = msg.step // msg.width  # bytes per pixel
    enc  = msg.encoding.lower()

    if enc == "rgb8":
        return data.reshape(msg.height, msg.width, 3)
    elif enc == "bgr8":
        return data.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
    elif enc == "mono8":
        return cv2.cvtColor(data.reshape(msg.height, msg.width), cv2.COLOR_GRAY2RGB)
    elif enc in ("yuyv", "yuv422", "yuv422_yuy2"):
        return cv2.cvtColor(data.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2RGB_YUYV)
    else:
        # fallback: step으로 채널 수 추정
        img = data.reshape(msg.height, msg.width, bpp)
        if bpp == 2:
            return cv2.cvtColor(img, cv2.COLOR_YUV2RGB_YUYV)
        elif bpp == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        return img


class ArmsDetectionNode(Node):
    def __init__(self):
        super().__init__("arms_detection_node")

        self.get_logger().info(f"Loading YOLO model from {MODEL_PATH} ...")
        self.model = YOLO(MODEL_PATH, task="detect")
        self.get_logger().info("Model loaded.")

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image, "/arms/image_raw", self.cb_image, best_effort_qos
        )
        self.pub = self.create_publisher(DetectionArray, "/arms/detections", 10)

        self.get_logger().info("arms_detection_node ready.")

    def cb_image(self, msg: Image):
        try:
            img = imgmsg_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        results = self.model.predict(img, conf=CONFIDENCE, iou=IOU, verbose=False)

        det_msg = DetectionArray()
        det_msg.header = msg.header

        h, w = img.shape[:2]
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            bbox = BoundingBox()
            bbox.x_center = float((x1 + x2) / 2 / w)
            bbox.y_center = float((y1 + y2) / 2 / h)
            bbox.width = float((x2 - x1) / w)
            bbox.height = float((y2 - y1) / h)
            bbox.confidence = float(box.conf[0])
            bbox.class_id = int(box.cls[0])
            bbox.class_name = self.model.names[int(box.cls[0])]
            det_msg.detections.append(bbox)

        self.pub.publish(det_msg)


def main():
    rclpy.init()
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
