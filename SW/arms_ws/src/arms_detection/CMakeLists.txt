cmake_minimum_required(VERSION 3.8)
project(arms_detection)

find_package(ament_cmake REQUIRED)

# OpenCV detection node (SITL / 개발용)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/arms_opencv_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install docker build context files for reference
install(DIRECTORY docker
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
