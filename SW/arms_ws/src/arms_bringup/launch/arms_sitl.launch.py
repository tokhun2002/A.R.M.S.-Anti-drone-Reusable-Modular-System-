"""
SITL launch: arms_control + arms_ui
- MAVLink via UDP (PX4 SITL default)
- GPIO disabled
- Camera/distance come from ros_gz_bridge (run separately)
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

    # SITL overrides: UDP connection, GPIO disabled
    sitl_overrides = {
        "mavlink.connection": "udp:127.0.0.1:14550",
        "gpio.enabled": False,
    }

    return LaunchDescription([
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(control_config), sitl_overrides],
        ),
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
        ),
    ])
