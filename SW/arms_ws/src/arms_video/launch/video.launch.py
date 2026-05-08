from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# sudo apt-get install ros-humble-usb-cam
def generate_launch_description():
    config = Path(get_package_share_directory("arms_video")) / "config" / "video_params.yaml"

    return LaunchDescription([
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="arms_video_node",
            output="screen",
            parameters=[str(config)],
            remappings=[
                ("image_raw", "/arms/image_raw"),
                ("camera_info", "/arms/camera_info"),
            ],
        ),
    ])
