"""
A.R.M.S. Detection Node — runs inside the Docker container.

Subscribes : /arms/image_raw  (sensor_msgs/Image)
Publishes  : /arms/detections (arms_msgs/DetectionArray)

Model path is read from the env var ARMS_MODEL (default: /models/best.engine).
TensorRT engine 추론 (YOLOv11 출력 포맷: [1, 4+nc, 8400])
"""

import os
import cv2
import numpy as np
import torch
import torchvision
import tensorrt as trt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from arms_msgs.msg import BoundingBox, DetectionArray

MODEL_PATH  = os.environ.get("ARMS_MODEL", "/models/best.engine")
CONFIDENCE  = float(os.environ.get("ARMS_CONF", "0.5"))
IOU         = float(os.environ.get("ARMS_IOU",  "0.45"))
CLASS_NAMES = os.environ.get("ARMS_CLASSES", "balloon").split(",")

INPUT_SIZE  = 640  # YOLOv11 default


# ──────────────────────────────────────────────
# TensorRT 추론 클래스
# ──────────────────────────────────────────────

class TRTInference:
    def __init__(self, engine_path: str):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # I/O 텐서 이름
        self.input_name  = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)

        # GPU 버퍼 사전 할당
        in_shape  = tuple(self.engine.get_tensor_shape(self.input_name))   # (1,3,640,640)
        out_shape = tuple(self.engine.get_tensor_shape(self.output_name))  # (1,4+nc,8400)
        self.input_buf  = torch.zeros(in_shape,  dtype=torch.float32, device="cuda")
        self.output_buf = torch.zeros(out_shape, dtype=torch.float32, device="cuda")

        self.context.set_tensor_address(self.input_name,  self.input_buf.data_ptr())
        self.context.set_tensor_address(self.output_name, self.output_buf.data_ptr())

    def preprocess(self, img: np.ndarray):
        """HxWx3 uint8 RGB -> (1,3,640,640) float32, 원본 크기 반환"""
        orig_h, orig_w = img.shape[:2]
        resized = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        tensor  = resized.transpose(2, 0, 1).astype(np.float32) / 255.0
        return tensor[np.newaxis], orig_w, orig_h

    def postprocess(self, output: np.ndarray, conf_thresh: float, iou_thresh: float):
        """
        output : (1, 4+nc, 8400)
        반환   : list of (x_center_norm, y_center_norm, w_norm, h_norm, conf, class_id)
                 좌표는 [0,1] 정규화
        """
        out = output[0].T  # (8400, 4+nc)
        boxes_raw  = out[:, :4]          # cx, cy, w, h  (640 스케일)
        class_conf = out[:, 4:]          # (8400, nc)

        class_ids   = np.argmax(class_conf, axis=1)
        confidences = class_conf[np.arange(len(class_conf)), class_ids]

        mask = confidences > conf_thresh
        if not mask.any():
            return []

        boxes_raw   = boxes_raw[mask]
        confidences = confidences[mask]
        class_ids   = class_ids[mask]

        # cx,cy,w,h → x1,y1,x2,y2 (NMS용)
        x1 = boxes_raw[:, 0] - boxes_raw[:, 2] / 2
        y1 = boxes_raw[:, 1] - boxes_raw[:, 3] / 2
        x2 = boxes_raw[:, 0] + boxes_raw[:, 2] / 2
        y2 = boxes_raw[:, 1] + boxes_raw[:, 3] / 2

        boxes_t  = torch.tensor(np.stack([x1, y1, x2, y2], axis=1), dtype=torch.float32)
        scores_t = torch.tensor(confidences, dtype=torch.float32)
        keep     = torchvision.ops.nms(boxes_t, scores_t, iou_thresh).numpy()

        results = []
        for i in keep:
            results.append((
                float(boxes_raw[i, 0] / INPUT_SIZE),  # x_center norm
                float(boxes_raw[i, 1] / INPUT_SIZE),  # y_center norm
                float(boxes_raw[i, 2] / INPUT_SIZE),  # w norm
                float(boxes_raw[i, 3] / INPUT_SIZE),  # h norm
                float(confidences[i]),
                int(class_ids[i]),
            ))
        return results

    def infer(self, img: np.ndarray, conf_thresh: float, iou_thresh: float):
        tensor, orig_w, orig_h = self.preprocess(img)
        self.input_buf.copy_(torch.from_numpy(tensor))
        self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        torch.cuda.synchronize()
        output = self.output_buf.cpu().numpy()
        return self.postprocess(output, conf_thresh, iou_thresh)


# ──────────────────────────────────────────────
# ROS2 노드
# ──────────────────────────────────────────────

def imgmsg_to_numpy(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → HxWx3 uint8 RGB (cv_bridge 없이)"""
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding, 3)
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding == "bgr8":
        img = img[:, :, ::-1].copy()
    return img


class ArmsDetectionNode(Node):
    def __init__(self):
        super().__init__("arms_detection_node")

        self.get_logger().info(f"Loading TensorRT engine from {MODEL_PATH} ...")
        self.trt = TRTInference(MODEL_PATH)
        self.get_logger().info("Engine loaded.")

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

        detections = self.trt.infer(img, CONFIDENCE, IOU)

        det_msg = DetectionArray()
        det_msg.header = msg.header

        for cx, cy, w, h, conf, cls_id in detections:
            bbox = BoundingBox()
            bbox.x_center   = cx
            bbox.y_center   = cy
            bbox.width      = w
            bbox.height     = h
            bbox.confidence = conf
            bbox.class_id   = cls_id
            bbox.class_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else str(cls_id)
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
