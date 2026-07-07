"""
arms_sitl_flying.launch.py — SITL "날아다니는 풍선 요격" 올인원 런치
이거 하나로 다음을 전부 띄운다 (PX4 SITL 제외, 그건 무거워서 따로):
  - gz 카메라 브리지        (/arms/image_raw)
  - gz ray 센서 브리지      (/arms/scan_raw)
  - pose_relay.py           (/arms/target_pose, /arms/drone_pose)  ← [옵션B] 좌표 릴레이
  - arms_control_node       (상태머신 + PID + MAVLink)
  - arms_ui_node            (OpenCV 영상 오버레이)
  - arms_detection_node     (융합검출: HSV + absdiff, YOLO 선택)
  - balloon_referee.py      (풍선 비행 + 명중 연출)
  - arms_command_node       (컨트롤 패널 GUI)

  - arms_panel.py           (컨트롤 패널 GUI)
전제: 환경변수 ARMS_SW 가 SW 폴더(=tools, simulation 들어있는 곳)를 가리켜야 함.
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )
    sw_dir = os.environ.get("ARMS_SW", "")
    tools = Path(sw_dir) / "tools" if sw_dir else None
    actions = [
        # 카메라 브리지
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="arms_video_node", output="screen",
            arguments=["/arms_drone/upward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[("/arms_drone/upward_camera/image", "/arms/image_raw")],
        ),
        # ray 센서 브리지
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_scan_bridge", output="screen",
            arguments=["/arms_drone/upward_ray/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            remappings=[("/arms_drone/upward_ray/scan", "/arms/scan_raw")],
        ),
        # 융합 검출 노드
        Node(
            package="arms_detection", executable="arms_detection_node",
            name="arms_detection_node", output="screen",
        ),
        # 제어
        Node(
            package="arms_control", executable="arms_control_node",
            name="arms_control_node", output="screen",
            parameters=[str(control_config),
                        {"mavlink.connection": "udp://:14540",
                         "gpio.enabled": False}],
        ),
        # UI 오버레이
        Node(
            package="arms_ui", executable="arms_ui_node",
            name="arms_ui_node", output="screen",
        ),
    ]

    actions.append(
        Node(
            package="arms_command", executable="arms_command_node",
            name="arms_command_node", output="screen",
        ),
    )

    if tools and tools.is_dir():
        actions += [
            # [옵션B] 좌표 릴레이 — gz dynamic_pose → /arms/target_pose, /arms/drone_pose
            ExecuteProcess(cmd=["python3", str(tools / "pose_relay.py")], output="screen"),
            ExecuteProcess(cmd=["python3", str(tools / "fusion_detector.py")], output="screen"),
            ExecuteProcess(cmd=["python3", str(tools / "balloon_referee.py")], output="screen"),
        )
    else:
        actions.append(LogInfo(msg=(
            "[arms_sitl_flying] ARMS_SW 환경변수가 안 잡혀서 풍선을 못 띄웠음. "
            "run_arms.sh 를 쓰거나 'export ARMS_SW=<SW경로>' 후 다시 실행.")))
    return LaunchDescription(actions)
