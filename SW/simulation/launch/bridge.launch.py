"""
ros_gz_bridge launch for A.R.M.S. SITL.

Bridges Gazebo topics to ROS2:
  /arms_drone/upward_camera/image  →  /arms/image_raw  (sensor_msgs/Image)
  /arms_drone/upward_ray/scan      →  /arms/distance   (sensor_msgs/Range)

Run this after starting PX4 SITL + Gazebo.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_params = [
        # Camera: Gazebo Image → ROS2 Image
        "/arms_drone/upward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        # Ray sensor: Gazebo LaserScan → ROS2 LaserScan (then remapped to Range in the node)
        "/arms_drone/upward_ray/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]

    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_ros2_bridge",
            output="screen",
            arguments=bridge_params,
            remappings=[
                ("/arms_drone/upward_camera/image", "/arms/image_raw"),
                # LaserScan → converted to Range inside arms_control_node
                ("/arms_drone/upward_ray/scan", "/arms/scan_raw"),
            ],
        ),
    ])
