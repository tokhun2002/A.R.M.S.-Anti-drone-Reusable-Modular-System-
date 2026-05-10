from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="arms_video_node",
            output="screen",
            arguments=[
                "/arms_drone/upward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            remappings=[
                ("/arms_drone/upward_camera/image", "/arms/image_raw"),
            ],
        ),
    ])
