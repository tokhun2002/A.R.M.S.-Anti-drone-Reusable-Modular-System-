"""
Full real-hardware launch: arms_video + arms_command + arms_detection + arms_control + arms_ui
- arms_command (ESP32 UART) → /arms/command → arms_control
- arms_control → CRSF serial (crsf.port 파라미터) → ELRS TX → [RF] → ELRS RX → FC
- Detection(YOLO+HSV+absdiff)은 GPU 도커 컨테이너로 동작. 이 런치가 'docker compose
  up -d' 로 자동 기동한다(멱등: 안 떠 있으면 띄우고, 떠 있으면 그대로). start_detection:=false
  로 끌 수 있음. 도커를 못 쓰거나 compose 를 못 찾으면 경고만 남기고 나머지는 계속 뜬다.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _find_detection_compose():
    """실기체 detection 컨테이너 compose 경로 탐색.
    compose 의 build context 가 소스트리 기준(../..)이라 반드시 '소스' 위치에서 실행해야
    하므로, ARMS_SW 환경변수 → 설치 share 에서 워크스페이스 역산 순으로 소스 경로를 찾는다."""
    rel_src = "src/arms_detection/docker/docker-compose.jetson.yml"
    sw = os.environ.get("ARMS_SW")
    if sw:
        p = Path(sw) / "arms_ws" / rel_src
        if p.exists():
            return str(p)
    try:
        share = Path(get_package_share_directory("arms_bringup"))
        for base in [share, *share.parents]:
            c = base / rel_src
            if c.exists():
                return str(c)
    except Exception:
        pass
    return ""


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
    detection_compose = _find_detection_compose()

    actions = [
        DeclareLaunchArgument("crsf_port", default_value="/dev/ttyTHS1",
                              description="ELRS TX serial port (CRSF output, 실기체 UART)"),
        DeclareLaunchArgument("start_detection", default_value="true",
                              description="detection(YOLO) 도커 컨테이너 자동 기동 여부"),
    ]

    # 검출 컨테이너 자동 기동 (docker compose up -d — 멱등). 컨테이너의 arms_detection_node
    # 가 /arms/detections 를 발행하므로 호스트에선 detection 노드를 따로 띄우지 않는다.
    if detection_compose:
        actions.append(ExecuteProcess(
            cmd=["docker", "compose", "-f", detection_compose, "up", "-d"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_detection")),
        ))
    else:
        actions.append(LogInfo(msg=(
            "[arms] detection compose 파일을 못 찾음 → 컨테이너 자동 기동 생략. "
            "ARMS_SW 를 SW 폴더로 export 하거나 수동으로 'docker compose up -d' 하세요.")))

    actions += [
        # Video capture (arms_video_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(video_launch)),
        ),
        # 실기체 조종기 입력 (ESP32 UART → /arms/command)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(command_launch)),
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
    ]

    return LaunchDescription(actions)
