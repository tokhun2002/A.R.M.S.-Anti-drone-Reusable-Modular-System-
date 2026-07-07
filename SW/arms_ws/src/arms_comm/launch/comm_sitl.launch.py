from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arms_comm",
            executable="arms_comm_sitl_node",
            name="arms_comm_sitl_node",
            output="screen",
            parameters=[{
                "connection": "udp://:14540",
                "max_angle_deg": 35.0,
                "send_rate_hz": 50.0,
            }],
        ),
    ])
