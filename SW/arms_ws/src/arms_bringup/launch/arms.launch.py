"""
Full real-hardware launch: arms_video + arms_command + arms_detection + arms_control + arms_ui
- arms_command (ESP32 UART) → /arms/command → arms_control
- arms_control → CRSF serial (crsf.port 파라미터) → ELRS TX → [RF] → ELRS RX → FC
- Detection: start separately via docker compose up
    cd arms_detection/docker && docker compose -f docker-compose.jetson.yml up
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_dir = Path(get_package_share_directory("arms_control")) / "config"
    control_config = control_dir / "control_params.yaml"
    # 실기체 CRSF 오버레이: crsf.port=/dev/ttyTHS1, crsf.baud=400000
    crsf_hw_config = control_dir / "crsf_hw.yaml"
    video_launch = (
        Path(get_package_share_directory("arms_video")) / "launch" / "video.launch.py"
    )
    command_launch = (
        Path(get_package_share_directory("arms_command")) / "launch" / "command.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("crsf_port", default_value="/dev/ttyTHS1",
                              description="ELRS TX serial port (CRSF output, 실기체 UART)"),

        # Video capture (arms_video_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(video_launch)),
        ),
        # 실기체 조종기 입력 (ESP32 UART → /arms/command)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(command_launch)),
        ),
        # 검출 노드
        Node(
            package="arms_detection", executable="arms_detection_node",
            name="arms_detection_node", output="screen",
        ),
        # 제어 (상태머신 + PID + CRSF 시리얼 출력 → ELRS TX → FC)
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[
                str(control_config),
                str(crsf_hw_config),                              # port + baud (실기체 확정값)
                {"crsf.port": LaunchConfiguration("crsf_port")},  # 필요시 포트만 오버라이드
            ],
        ),
        # OpenCV UI
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
        ),
    ])
