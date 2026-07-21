"""
샘플 비디오 replay 기반 풀스택 런치 (arms.launch.py 의 테스트용 변형).

실기체 bringup(arms.launch.py) 과 동일하게 detection + control + ui 를 구동하되,
영상 소스만 v4l2_camera 대신 저장된 샘플 비디오(video_replay)로 대체한다.
카메라/FC 없이도 저장 영상으로 검출·추적·UI 파이프라인을 검증할 수 있다.

  ros2 launch arms_bringup arms_replay.launch.py
  ros2 launch arms_bringup arms_replay.launch.py video_path:=/경로/sample2.mov

- Detection(YOLO): 별도로 docker compose up 시 샘플에도 YOLO 적용됨.
"""

import os
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
    replay_launch = (
        Path(get_package_share_directory("arms_video")) / "launch" / "video_replay.launch.py"
    )

    # 기본 샘플 영상: 위로 올라가며 sample_viedo 를 찾는다 (심링크/복사 install 모두 대응)
    here = Path(os.path.realpath(__file__))
    default_video = next(
        (str(p / "sample_viedo" / "sample1.mov") for p in here.parents
         if (p / "sample_viedo" / "sample1.mov").exists()),
        str(here.parents[4] / "sample_viedo" / "sample1.mov"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("crsf_port", default_value="/dev/ttyUSB0",
                              description="ELRS TX serial port (CRSF output)"),
        DeclareLaunchArgument("video_path", default_value=default_video,
                              description="재생할 샘플 비디오 경로 (기본: sample1.mov)"),
        DeclareLaunchArgument("publish_rate", default_value="30.0",
                              description="발행 fps"),

        # 영상 소스: 샘플 비디오 replay (→ /arms/image_raw)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(replay_launch)),
            launch_arguments={
                "video_path": LaunchConfiguration("video_path"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }.items(),
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
    ])
