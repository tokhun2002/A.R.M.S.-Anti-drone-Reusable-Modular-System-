"""
SITL launch: arms_video (gz bridge) + arms_detection + arms_control + arms_comm_sitl + arms_ui
- PID 제어 명령 → /arms/ctrl_cmd → arms_comm_sitl → pymavlink RC_CHANNELS_OVERRIDE → PX4 Stabilized
- 카메라 브리지: arms_video/launch/video_sitl.launch.py
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
        # Fusion detector (HSV + absdiff, YOLO 도커 연동 시 use_yolo=true)
        Node(
            package="arms_detection",
            executable="arms_detection_node",
            name="arms_detection_node",
            output="screen",
        ),
        # 제어 (순수 PID → /arms/ctrl_cmd 발행)
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(control_config)],
        ),
        # 통신 (pymavlink → PX4 Stabilized mode)
        Node(
            package="arms_comm",
            executable="arms_comm_sitl_node",
            name="arms_comm_sitl_node",
            output="screen",
            parameters=[{
                "connection": "udpin:0.0.0.0:14540",
                "max_angle_deg": 35.0,
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
