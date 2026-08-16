from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 실기체 FPV 영상 수신.
#
# FPV 아날로그 → USB 캡처 동글(MS210x/EasierCAP, A2539 등)을 v4l2_camera(C++)로
# 캡처해 /arms/image_raw 로 발행한다. usb_cam 은 이런 stepwise 프레임 간격
# 장치에서 포맷 열거에 실패하므로 v4l2_camera 를 쓴다.
#
# 파라미터/캘리브레이션은 config/ 의 yaml 에서 로드한다.
# 아날로그 신호 락에 ~5초 걸릴 수 있음 (그동안 검은 화면은 정상).
def generate_launch_description():
    config = Path(get_package_share_directory("arms_video")) / "config" / "video_params.yaml"

    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="arms_video_node",
            output="screen",
            parameters=[str(config)],
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
    ])
