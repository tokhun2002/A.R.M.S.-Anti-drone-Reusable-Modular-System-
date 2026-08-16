"""
샘플 비디오 replay 기반 풀스택 런치 (arms.launch.py 의 테스트용 변형).

실기체 bringup(arms.launch.py) 과 동일하게 detection + control + ui 를 구동하되,
영상 소스만 v4l2_camera 대신 저장된 샘플 비디오(video_replay)로 대체한다.
카메라/FC 없이도 저장 영상으로 검출·추적·UI 파이프라인을 검증할 수 있다.

  ros2 launch arms_bringup arms_replay.launch.py
  ros2 launch arms_bringup arms_replay.launch.py video_path:=/경로/sample2.mov

- Detection(YOLO+HSV+absdiff): arms.launch.py 와 동일하게 GPU 도커 컨테이너로
  동작한다. 이 런치가 'docker compose up -d' 로 자동 기동한다(멱등). start_detection:=false
  로 끌 수 있고, 로드할 가중치는 model:= 로 고른다(기본 balloon_camera.pt).
    ros2 launch arms_bringup arms_replay.launch.py                              # 카메라모델(기본)
    ros2 launch arms_bringup arms_replay.launch.py model:=/models/balloon.engine
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
    """현재 장비에 맞는 detection compose 경로 탐색.

    Jetson은 전용 런타임을 쓰고, 일반 x86 노트북은 CPU/CUDA 공용 laptop
    이미지를 사용한다. ARMS_DETECTION_PLATFORM=jetson|laptop 으로 강제할 수 있다.

    compose 의 build context 가 소스트리 기준(../..)이라 반드시 '소스' 위치에서 실행해야
    하므로, ARMS_SW 환경변수 → 설치 share 에서 워크스페이스 역산 순으로 소스 경로를 찾는다."""
    platform = os.environ.get("ARMS_DETECTION_PLATFORM", "").strip().lower()
    if platform not in {"jetson", "laptop"}:
        platform = "jetson" if Path("/etc/nv_tegra_release").exists() else "laptop"
    rel_src = f"src/arms_detection/docker/docker-compose.{platform}.yml"
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
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )
    replay_launch = (
        Path(get_package_share_directory("arms_video")) / "launch" / "video_replay.launch.py"
    )
    # 조종기: SITL 가상 조종기(tkinter GUI). 실기체 ESP32 대신 클릭으로 arm/모드/kill/launch 발행.
    command_launch = (
        Path(get_package_share_directory("arms_command")) / "launch" / "command_sitl.launch.py"
    )
    detection_compose = _find_detection_compose()

    # 기본 샘플 영상: 위로 올라가며 sample_viedo 를 찾는다 (심링크/복사 install 모두 대응)
    here = Path(os.path.realpath(__file__))
    default_video = next(
        (str(p / "sample_viedo" / "sample1.mov") for p in here.parents
         if (p / "sample_viedo" / "sample1.mov").exists()),
        str(here.parents[4] / "sample_viedo" / "sample1.mov"),
    )

    actions = [
        DeclareLaunchArgument("crsf_port", default_value="/dev/ttyUSB0",
                              description="ELRS TX serial port (CRSF output)"),
        DeclareLaunchArgument("video_path", default_value=default_video,
                              description="재생할 샘플 비디오 경로 (기본: sample1.mov)"),
        DeclareLaunchArgument("publish_rate", default_value="30.0",
                              description="발행 fps"),
        DeclareLaunchArgument("start_detection", default_value="true",
                              description="detection(YOLO) 도커 컨테이너 자동 기동 여부"),
        DeclareLaunchArgument(
            "model", default_value="/models/balloon_camera.engine",
            description="detection 컨테이너가 로드할 가중치(컨테이너 내부 경로). "
                        "기본=/models/balloon_camera.engine (FP16 TensorRT)"),
    ]

    # 검출: arms.launch.py 와 동일하게 도커 컨테이너를 자동 기동한다(멱등). 컨테이너의
    # arms_detection_node 가 /arms/detections 를 발행하므로 호스트에서 detection 노드를
    # 따로 띄우지 않는다(둘이 발행하면 충돌). model:= → ARMS_MODEL 로 주입.
    if detection_compose:
        actions.append(ExecuteProcess(
            cmd=["docker", "compose", "-f", detection_compose, "up", "-d"],
            output="screen",
            additional_env={"ARMS_MODEL": LaunchConfiguration("model")},
            condition=IfCondition(LaunchConfiguration("start_detection")),
        ))
    else:
        actions.append(LogInfo(msg=(
            "[arms_replay] detection compose 파일을 못 찾음 → 컨테이너 자동 기동 생략. "
            "ARMS_SW 를 SW 폴더로 export 하거나 수동으로 'docker compose up -d' 하세요.")))

    actions += [
        # 영상 소스: 샘플 비디오 replay (→ /arms/image_raw)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(replay_launch)),
            launch_arguments={
                "video_path": LaunchConfiguration("video_path"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }.items(),
        ),
        # 조종기: SITL 가상 조종기(tkinter GUI) → /arms/command. arm/모드/kill/launch 입력.
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
                {"crsf.port": LaunchConfiguration("crsf_port")},
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
