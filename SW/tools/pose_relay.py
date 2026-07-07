#!/usr/bin/env python3
"""
pose_relay.py — [옵션B] 좌표 기반 요격용 위치 릴레이

Gazebo dynamic_pose/info (gz transport13, Pose_V) 를 직접 읽어서
이름으로 red_ball(풍선)과 arms_drone_0(드론)을 정확히 골라 ROS 토픽으로 발행.

발행:
  /arms/target_pose  (geometry_msgs/PoseStamped)  — 풍선 월드좌표 (Gazebo ENU)
  /arms/drone_pose   (geometry_msgs/PoseStamped)  — 드론  월드좌표 (Gazebo ENU)

좌표계: Gazebo ENU(x=동,y=북,z=상). NED 변환은 control 노드에서.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

TARGET_NAME = "red_ball"
DRONE_NAME = "arms_drone_0"
GZ_TOPIC = "/world/arms_sitl/dynamic_pose/info"


class PoseRelay(Node):
    def __init__(self):
        super().__init__("pose_relay")
        self.pub_target = self.create_publisher(PoseStamped, "/arms/target_pose", 10)
        self.pub_drone = self.create_publisher(PoseStamped, "/arms/drone_pose", 10)

        self.gz = GzNode()
        ok = self.gz.subscribe(Pose_V, GZ_TOPIC, self.on_pose_v)
        if ok:
            self.get_logger().info(
                f"pose_relay ready. gz '{GZ_TOPIC}' 구독 → "
                f"/arms/target_pose({TARGET_NAME}), /arms/drone_pose({DRONE_NAME})")
        else:
            self.get_logger().error(f"gz 토픽 구독 실패: {GZ_TOPIC}")
        self._warned = False

    def on_pose_v(self, msg: Pose_V):
        now = self.get_clock().now().to_msg()
        found_t = found_d = False
        for p in msg.pose:
            if p.name == TARGET_NAME:
                self.pub_target.publish(self._to_stamped(p, now))
                found_t = True
            elif p.name == DRONE_NAME:
                self.pub_drone.publish(self._to_stamped(p, now))
                found_d = True
        if not (found_t and found_d) and not self._warned:
            names = [p.name for p in msg.pose]
            self.get_logger().warn(f"이름 매칭 일부 실패. 들어온 이름들: {names}")
            self._warned = True

    @staticmethod
    def _to_stamped(p, stamp):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = "world"
        ps.pose.position.x = p.position.x
        ps.pose.position.y = p.position.y
        ps.pose.position.z = p.position.z
        ps.pose.orientation.w = 1.0
        return ps


def main():
    rclpy.init()
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
