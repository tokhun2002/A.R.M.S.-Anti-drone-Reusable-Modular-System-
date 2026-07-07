from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arms_command",
            executable="arms_command_gpio_node",
            name="arms_command_node",
            output="screen",
        ),
    ])
