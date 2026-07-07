"""실기체: ADS1115/GPIO 조종기 입력 노드."""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = (
        Path(get_package_share_directory("arms_command")) / "config" / "controller_input_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="arms_command",
            executable="controller_input_node",
            name="controller_input_node",
            output="screen",
            parameters=[str(config)],
        ),
    ])
