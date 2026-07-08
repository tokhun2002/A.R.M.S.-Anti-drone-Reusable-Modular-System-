from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"

    return LaunchDescription([
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(config)],
        ),
        Node(
            package="arms_control",
            executable="sitl_bridge_node",
            name="arms_sitl_bridge_node",
            output="screen",
            parameters=[{
                "connection":   "udpin:0.0.0.0:14540",
                "crsf_port":    "/tmp/crsf_rx",
                "send_rate_hz": 50.0,
            }],
        ),
    ])
