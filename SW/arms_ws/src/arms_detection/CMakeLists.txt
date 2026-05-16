cmake_minimum_required(VERSION 3.8)
project(arms_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arms_msgs REQUIRED)

# OpenCV detection node (SITL / 개발용)
install(PROGRAMS
  arms_opencv_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install docker build context files for reference
install(DIRECTORY docker
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
