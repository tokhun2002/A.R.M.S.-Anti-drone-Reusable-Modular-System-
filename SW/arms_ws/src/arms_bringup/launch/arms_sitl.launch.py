"""
SITL launch: arms_video (gz bridge) + arms_detection + arms_control + arms_sitl_bridge + arms_ui
- arms_control → CRSF serial → arms_sitl_bridge → MAVLink RC_CHANNELS_OVERRIDE → PX4 Stabilized
- 카메라 브리지: arms_video/launch/video_sitl.launch.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
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
    pj_layout = Path(get_package_share_directory("arms_bringup")) / "config" / "sitl_debug.xml"
    pj_cmd = ["ros2", "run", "plotjuggler", "plotjuggler", "--ros", "--buffer_size", "60"]
    if pj_layout.exists():
        pj_cmd += ["--layout", str(pj_layout)]

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
        # 제어 (상태머신 + PID + CRSF 시리얼 출력)
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(control_config)],
        ),
        # SITL 브리지 (CRSF → MAVLink RC_CHANNELS_OVERRIDE → PX4)
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
        # OpenCV UI
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
        ),
        # PlotJuggler — 노드 startup 후 5초 지연해서 실행
        TimerAction(period=5.0, actions=[
            ExecuteProcess(cmd=pj_cmd, output="screen"),
        ]),
    ])
