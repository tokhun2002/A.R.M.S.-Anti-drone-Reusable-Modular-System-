"""
arms_ui_node — OpenCV overlay display (skeleton).

Subscribes to:
  /arms/image_raw      sensor_msgs/Image
  /arms/detections     arms_msgs/DetectionArray
  /arms/mission_state  arms_msgs/MissionState

Displays:
  - Live camera feed
  - Bounding boxes with confidence
  - Frame center crosshair
  - Mission state overlay (state name, lock progress bar, error vector)
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from arms_msgs.msg import DetectionArray, MissionState
from geometry_msgs.msg import Vector3

# State → BGR color mapping
STATE_COLORS = {
    "IDLE":   (128, 128, 128),
    "SEARCH": (0,   200, 255),
    "LOCK":   (0,   140, 255),
    "BOOST":  (0,   110, 255),
    "TRACK":  (0,   0,   220),
    "FIRE":   (0,   0,   220),
    "RTL":    (255, 180, 0),
}


class ArmsUINode(Node):
    def __init__(self):
        super().__init__("arms_ui_node")
        self._bridge = CvBridge()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, "/arms/image_raw", self._cb_image, best_effort_qos)
        self.create_subscription(DetectionArray, "/arms/detections", self._cb_detections, 10)
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)
        self.create_subscription(Vector3, "/arms/control_debug", self._cb_debug, 10)

        self._latest_detections = DetectionArray()
        self._latest_state = MissionState()
        self._latest_cmd = Vector3()

        # 창 크기 조절 가능 (검출 해상도는 작게 유지, 화면만 크게)
        cv2.namedWindow("A.R.M.S.", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("A.R.M.S.", 960, 720)   # 기본 표시 크기 (마우스로 조절 가능)

        self.get_logger().info("arms_ui_node started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_detections(self, msg: DetectionArray):
        self._latest_detections = msg

    def _cb_state(self, msg: MissionState):
        self._latest_state = msg

    def _cb_debug(self, msg: Vector3):
        self._latest_cmd = msg

    def _cb_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        self._draw_overlay(frame)
        cv2.imshow("A.R.M.S.", frame)
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _draw_overlay(self, frame):
        h, w = frame.shape[:2]
        state = self._latest_state.state or "IDLE"
        color = STATE_COLORS.get(state, (255, 255, 255))

        # --- Bounding boxes ---
        for det in self._latest_detections.detections:
            cx = int(det.x_center * w)
            cy = int(det.y_center * h)
            bw = int(det.width * w)
            bh = int(det.height * h)
            x1, y1 = cx - bw // 2, cy - bh // 2
            x2, y2 = cx + bw // 2, cy + bh // 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{det.confidence:.2f}", (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # --- Crosshair at frame center ---
        cx_f, cy_f = w // 2, h // 2
        cv2.line(frame, (cx_f - 20, cy_f), (cx_f + 20, cy_f), (0, 255, 0), 1)
        cv2.line(frame, (cx_f, cy_f - 20), (cx_f, cy_f + 20), (0, 255, 0), 1)

        # --- State label ---
        cv2.putText(frame, state, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        # --- 현재 P (TRACK/FIRE 때 시간 램프 값 표시) ---
        if state in ("TRACK", "FIRE"):
            cv2.putText(frame, f"P {self._latest_state.kp_now:.0f}", (10, h - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

        # --- Lock progress bar (SEARCH / LOCK) ---
        if state in ("SEARCH", "LOCK"):
            lock_duration = 2.0  # TODO: read from param
            progress = min(1.0, self._latest_state.lock_elapsed_sec / lock_duration)
            bar_w = int(w * 0.4)
            bar_x, bar_y = 10, h - 20
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 10), (80, 80, 80), -1)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(bar_w * progress), bar_y + 10), color, -1)
            cv2.putText(frame, "LOCK", (bar_x, bar_y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # --- Error vector (LOCK / TRACK / FIRE) : 노란색 = 풍선 방향 ---
        if state in ("LOCK", "TRACK", "FIRE"):
            ex = int(self._latest_state.error_x * w)
            ey = int(self._latest_state.error_y * h)
            cv2.arrowedLine(frame, (cx_f, cy_f), (cx_f + ex, cy_f + ey),
                            (0, 255, 255), 2, tipLength=0.2)
            cv2.putText(frame, "ERR(target)", (cx_f + ex + 5, cy_f + ey),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            # --- 실제 나가는 제어 명령 : 빨강 = roll(가로)/pitch(세로) ---
            roll = self._latest_cmd.x
            pitch = self._latest_cmd.y
            thr = self._latest_cmd.z
            scale = 8  # 1도당 픽셀
            rx = int(roll * scale)
            ry = int(-pitch * scale)  # pitch +면 위로 표시
            cv2.arrowedLine(frame, (cx_f, cy_f), (cx_f + rx, cy_f + ry),
                            (0, 0, 255), 2, tipLength=0.2)
            cv2.putText(frame, "CMD(roll/pitch)", (cx_f + rx + 5, cy_f + ry + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # --- 숫자 디버그 텍스트 (좌상단) ---
            cv2.putText(frame, f"err  x={self._latest_state.error_x:+.2f} y={self._latest_state.error_y:+.2f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(frame, f"cmd  roll={roll:+.1f} pitch={pitch:+.1f} thr={thr:.2f}",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # --- State border ---
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), color, 3)


def main(args=None):
    rclpy.init(args=args)
    node = ArmsUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
