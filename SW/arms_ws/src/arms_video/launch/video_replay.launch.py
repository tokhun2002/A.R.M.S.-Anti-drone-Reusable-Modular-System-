import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 샘플 비디오 파일을 /arms/image_raw 로 스트리밍 (카메라 대신 시스템 테스트용).
# 표준 image_publisher(C++) 노드를 사용한다. (파이썬 replay 보다 발행이 가볍다)
#   설치:  sudo apt install ros-humble-image-publisher
#
# 기본 영상은 레포의 SW/sample_viedo/sample2.mov.
#   ros2 launch arms_video video_replay.launch.py video_path:=/경로/x.mov publish_rate:=30.0
def _setup(context, *args, **kwargs):
    # '~' / 상대경로도 받아들이도록 확장 (image_publisher 는 '~' 를 못 푼다)
    video_path = LaunchConfiguration("video_path").perform(context)
    video_path = os.path.abspath(os.path.expanduser(video_path))
    publish_rate = float(LaunchConfiguration("publish_rate").perform(context))

    return [
        Node(
            package="image_publisher",
            executable="image_publisher_node",
            name="arms_video_node",
            output="screen",
            parameters=[{
                "filename": video_path,
                "publish_rate": publish_rate,
            }],
            remappings=[
                ("image_raw", "/arms/image_raw"),
                # image_transport 압축 서브토픽은 베이스 리매핑이 안 따라간다
                # (기본이면 /image_raw/compressed 로 나감) → 명시적으로 리매핑해야
                # detection(도커)이 구독하는 /arms/image_raw/compressed 와 이름이 맞는다.
                ("image_raw/compressed", "/arms/image_raw/compressed"),
                ("image_raw/compressedDepth", "/arms/image_raw/compressedDepth"),
                ("image_raw/theora", "/arms/image_raw/theora"),
                ("camera_info", "/arms/camera_info"),
            ],
        ),
    ]


def _default_sample(name="sample2.mov"):
    # 이 런치 파일 위치에서 위로 올라가며 sample_viedo 디렉토리를 찾는다.
    # (install 이 심링크든 복사본이든 모두 대응)
    here = Path(os.path.realpath(__file__))
    for p in here.parents:
        cand = p / "sample_viedo" / name
        if cand.exists():
            return str(cand)
    return str(here.parents[4] / "sample_viedo" / name)  # fallback


def generate_launch_description():
    default_video = _default_sample()

    return LaunchDescription([
        DeclareLaunchArgument("video_path", default_value=default_video,
                              description="재생할 비디오 파일 경로 (~ / 상대경로 허용)"),
        DeclareLaunchArgument("publish_rate", default_value="30.0",
                              description="발행 fps"),
        OpaqueFunction(function=_setup),
    ])
