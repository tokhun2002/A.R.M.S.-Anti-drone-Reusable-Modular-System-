"""
SITL launch: ros_gz_bridge + arms_control + arms_ui
- MAVLink via UDP (PX4 SITL default)
- GPIO disabled
- Gazebo 카메라/거리 센서를 ROS2 토픽으로 브리지
- Detection: 별도로 docker compose up 필요
    cd arms_detection/docker && docker compose up
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )

    sitl_overrides = {
        "mavlink.connection": "udp:127.0.0.1:14550",
        "gpio.enabled": False,
    }

    bridge_params = [
        "/arms_drone/upward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/arms_drone/upward_ray/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]

    return LaunchDescription([
        # Gazebo ↔ ROS2 bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_ros2_bridge",
            output="screen",
            arguments=bridge_params,
            remappings=[
                ("/arms_drone/upward_camera/image", "/arms/image_raw"),
                ("/arms_drone/upward_ray/scan",     "/arms/scan_raw"),
            ],
        ),
        # State machine + PID + MAVLink
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(control_config), sitl_overrides],
        ),
        # OpenCV UI
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
        ),
    ])
