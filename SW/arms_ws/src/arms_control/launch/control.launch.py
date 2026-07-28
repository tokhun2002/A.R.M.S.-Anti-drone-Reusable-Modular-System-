from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = Path(get_package_share_directory("arms_control")) / "config"
    config = config_dir / "control_params.yaml"
    # 실제 하드웨어용 CRSF 포트/baud 오버레이 (ELRS TX 가 물린 UART).
    crsf_hw = config_dir / "crsf_hw.yaml"

    return LaunchDescription([
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(config), str(crsf_hw)],
        ),
    ])
