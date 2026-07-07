from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("max_angle_deg", default_value="35.0"),

        Node(
            package="arms_comm",
            executable="arms_comm_node",
            name="arms_comm_node",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "baud": 420000,
                "max_angle_deg": LaunchConfiguration("max_angle_deg"),
                "send_rate_hz": 50.0,
            }],
        ),
    ])
