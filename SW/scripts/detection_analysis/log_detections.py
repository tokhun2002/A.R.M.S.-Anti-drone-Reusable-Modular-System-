#!/usr/bin/env python3
"""
log_detections.py — /arms/detections 토픽을 1분간 CSV로 기록

출력: detections_<timestamp>.csv
  columns: time_sec, x_center, y_center, confidence, class_name

실행:
  source /opt/ros/humble/setup.bash
  source <arms_ws>/install/setup.bash
  python3 log_detections.py
"""

import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from arms_msgs.msg import DetectionArray

LOG_DURATION_SEC = 60.0


class DetectionLogger(Node):
    def __init__(self, out_path: Path):
        super().__init__("detection_logger")
        self._start = self.get_clock().now()
        self._rows = []
        self._done = False

        self.create_subscription(DetectionArray, "/arms/detections", self._cb, 10)
        self.create_timer(1.0, self._status)
        self.get_logger().info(f"로깅 시작 → {out_path}  ({LOG_DURATION_SEC:.0f}초)")
        self._out_path = out_path

    def _cb(self, msg: DetectionArray):
        if self._done:
            return

        now = self.get_clock().now()
        elapsed = (now - self._start).nanoseconds * 1e-9

        if elapsed > LOG_DURATION_SEC:
            self._finish()
            return

        if not msg.detections:
            return

        best = max(msg.detections, key=lambda d: d.confidence)
        self._rows.append((elapsed, best.x_center, best.y_center,
                           best.confidence, best.class_name))

    def _status(self):
        if self._done:
            return
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        remaining = max(0, LOG_DURATION_SEC - elapsed)
        self.get_logger().info(
            f"  {elapsed:5.1f}s 경과 | {len(self._rows):5d} 샘플 | 남은 시간 {remaining:.0f}s"
        )
        if elapsed > LOG_DURATION_SEC:
            self._finish()

    def _finish(self):
        if self._done:
            return
        self._done = True
        with open(self._out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_sec", "x_center", "y_center", "confidence", "class_name"])
            w.writerows(self._rows)
        self.get_logger().info(
            f"저장 완료: {self._out_path}  ({len(self._rows)} 샘플)"
        )
        raise SystemExit(0)


def main():
    rclpy.init()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(__file__).parent / f"detections_{ts}.csv"
    node = DetectionLogger(out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
