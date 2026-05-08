"""
Full real-hardware launch: arms_video + arms_control + arms_ui
- Detection: start separately via docker compose up
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
    video_config = (
        Path(get_package_share_directory("arms_video")) / "config" / "video_params.yaml"
    )

    hw_overrides = {
        "mavlink.connection": "/dev/ttyTHS1",
        "gpio.enabled": True,
    }

    return LaunchDescription([
        # Video capture (usb_cam)
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="arms_video_node",
            output="screen",
            parameters=[str(video_config)],
            remappings=[
                ("image_raw", "/arms/image_raw"),
                ("camera_info", "/arms/camera_info"),
            ],
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
