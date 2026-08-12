"""
arms_sitl.launch.py — SITL "날아다니는 풍선 요격" 올인원 런치
이거 하나로 다음을 전부 띄운다 (PX4 SITL 제외, 그건 무거워서 따로):
  - gz 카메라 브리지        (/arms/image_raw)
  - arms_detection_node     (융합검출: HSV + absdiff, YOLO 선택)
  - arms_control_node       (상태머신 + PID + CRSF 출력)
  - arms_sitl_bridge_node   (CRSF→MAVLink RC override → PX4)
  - arms_command_node       (가상 조종기: 스틱+버튼 → /arms/command)   [arms_command]
  - panel                   (튜닝/심판 콘솔)                          [arms_sim]
  - referee                 (표적 풍선/드론 비행 + 명중 판정)          [arms_sim]
  - arms_ui_node            (OpenCV 영상 오버레이)
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )
    pj_layout = Path(get_package_share_directory("arms_bringup")) / "config" / "sitl_debug.xml"
    pj_cmd = ["ros2", "run", "plotjuggler", "plotjuggler", "--buffer_size", "60"]
    if pj_layout.exists():
        pj_cmd += ["--layout", str(pj_layout)]

    actions = [
        # 카메라 브리지 — 상방 카메라 → /arms/image_raw
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="arms_video_up", output="screen",
            arguments=["/arms_drone/upward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[("/arms_drone/upward_camera/image", "/arms/image_raw")],
        ),
        # 융합 검출 노드
        Node(
            package="arms_detection", executable="arms_detection_node",
            name="arms_detection_node", output="screen",
        ),
        # 제어 (상태머신 + PID + CRSF 시리얼 출력)
        # SITL: socat PTY(/tmp/crsf_tx)로 출력 → sitl_bridge가 /tmp/crsf_rx에서 수신
        Node(
            package="arms_control", executable="arms_control_node",
            name="arms_control_node", output="screen",
            parameters=[str(control_config),
                        {"crsf.port": "/tmp/crsf_tx"}],
        ),
        # SITL 브리지 (CRSF → MAVLink RC_CHANNELS_OVERRIDE → PX4)
        Node(
            package="arms_control", executable="sitl_bridge_node",
            name="arms_sitl_bridge_node", output="screen",
            parameters=[{"connection":   "udpin:0.0.0.0:14540",
                         "crsf_port":    "/tmp/crsf_rx",
                         "send_rate_hz": 50.0}],
        ),
        # 가상 조종기 (스틱+버튼 → /arms/command). 실기체 물리 조종기의 SITL 쌍둥이.
        #   /arms/command 발행자는 이 노드 하나뿐 (레이스 방지).
        Node(
            package="arms_command", executable="arms_command_node",
            name="arms_command_node", output="screen",
        ),
        # 튜닝/심판 콘솔 (파라미터·referee 제어). SITL 전용, /arms/command 발행 안 함.
        Node(
            package="arms_sim", executable="panel",
            name="arms_panel_node", output="screen",
        ),
        # 표적 심판 (풍선/드론 비행 + 명중 판정). SITL 전용.
        #   node name = balloon_referee (panel 의 REFEREE_NODE 와 일치必).
        Node(
            package="arms_sim", executable="referee",
            name="balloon_referee", output="screen",
        ),
        # UI 오버레이
        Node(
            package="arms_ui", executable="arms_ui_node",
            name="arms_ui_node", output="screen",
        ),
    ]

    # PlotJuggler — 노드 startup 후 5초 지연해서 실행
    actions.append(TimerAction(period=5.0, actions=[
        ExecuteProcess(cmd=pj_cmd, output="screen"),
    ]))

    return LaunchDescription(actions)
