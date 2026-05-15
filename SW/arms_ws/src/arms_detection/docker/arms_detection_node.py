"""
A.R.M.S. Detection Node — runs inside the Docker container.

Subscribes : /arms/image_raw  (sensor_msgs/Image)
Publishes  : /arms/detections (arms_msgs/DetectionArray)

Model path is read from the env var ARMS_MODEL (default: /models/drone.pt).
"""

import os
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
    """Convert sensor_msgs/Image to HxWxC uint8 numpy array (no cv_bridge needed)."""
    dtype = np.uint8
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding, 3)
    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
    # YOLO expects RGB; sensor_msgs default is rgb8 from usb_cam / Gazebo bridge
    if msg.encoding == "bgr8":
        img = img[:, :, ::-1].copy()
    return img


class ArmsDetectionNode(Node):
    def __init__(self):
        super().__init__("arms_detection_node")

        self.get_logger().info(f"Loading YOLO model from {MODEL_PATH} ...")
        self.model = YOLO(MODEL_PATH)
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
