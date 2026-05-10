"""
Full real-hardware launch: arms_video + arms_control + arms_ui
- Detection: start separately via docker compose up
    cd arms_detection/docker && docker compose -f docker-compose.jetson.yml up
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
    video_launch = (
        Path(get_package_share_directory("arms_video")) / "launch" / "video.launch.py"
    )

    hw_overrides = {
        "mavlink.connection": "/dev/ttyTHS1",
        "gpio.enabled": True,
    }

    return LaunchDescription([
        # Video capture (arms_video_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(video_launch)),
        ),
        # State machine + PID + MAVLink
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(control_config), hw_overrides],
        ),
        # OpenCV UI
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
        ),
    ])
