"""
SITL launch (OpenCV 원 감지) : arms_video + arms_opencv_detection + arms_control + arms_ui

YOLO Docker 없이 동작 검증용.
HoughCircles 파라미터는 ros2 param set 으로 실시간 조정 가능.

    ros2 param set /arms_opencv_detection_node param2 20
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
        "mavlink.connection": "udp://:14540",
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
        # OpenCV 원 감지 노드 (YOLO 대체)
        Node(
            package="arms_detection",
            executable="arms_opencv_detection_node.py",
            name="arms_opencv_detection_node",
            output="screen",
            parameters=[{
                "blur_ksize":  11,
                "dp":          1.2,
                "min_dist":    50,
                "param1":      80,
                "param2":      30,
                "min_radius":  10,
                "max_radius":  200,
                "confidence":  0.8,
            }],
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
