"""
Full real-hardware launch: arms_video + arms_detection + arms_control + arms_comm + arms_ui
- 제어 명령 → /arms/ctrl_cmd → arms_comm → CRSF/ELRS → FC (Stabilized mode)
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
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )
    video_launch = (
        Path(get_package_share_directory("arms_video")) / "launch" / "video.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0",
                              description="ELRS TX serial port"),
        DeclareLaunchArgument("max_angle_deg", default_value="35.0",
                              description="FC max tilt angle (MC_ROLL_MAX)"),

        # Video capture (arms_video_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(video_launch)),
        ),
        # 제어 (순수 PID → /arms/ctrl_cmd 발행)
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(control_config)],
        ),
        # 통신 (pyserial CRSF → ELRS → FC Stabilized)
        Node(
            package="arms_comm",
            executable="arms_comm_node",
            name="arms_comm_node",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "baud": 420000,
                "max_angle_deg": LaunchConfiguration("max_angle_deg"),
                "send_rate_hz": 50.0,
            }],
        ),
        # OpenCV UI
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
        ),
    ])
