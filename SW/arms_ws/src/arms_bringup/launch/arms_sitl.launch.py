"""
SITL launch: arms_video (gz bridge) + arms_control + arms_ui
- MAVLink via UDP (PX4 SITL default)
- GPIO disabled
- 카메라 브리지: arms_video/launch/video_sitl.launch.py (arms_video_node)
- 거리 센서 브리지: 이 파일에서 직접 처리
- Detection: 별도로 docker compose up 필요
    cd arms_detection/docker && docker compose -f docker-compose.laptop.yml up
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )
    video_sitl_launch = (
        Path(get_package_share_directory("arms_video")) / "launch" / "video_sitl.launch.py"
    )

    sitl_overrides = {
        "mavlink.connection": "udp:127.0.0.1:14550",
        "gpio.enabled": False,
    }

    return LaunchDescription([
        # 카메라 브리지 (arms_video_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(video_sitl_launch)),
        ),
        # 거리 센서 브리지
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_scan_bridge",
            output="screen",
            arguments=[
                "/arms_drone/upward_ray/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            ],
            remappings=[
                ("/arms_drone/upward_ray/scan", "/arms/scan_raw"),
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
