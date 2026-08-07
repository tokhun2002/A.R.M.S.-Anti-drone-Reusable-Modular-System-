cmake_minimum_required(VERSION 3.14)
project(arms_control)

set(CMAKE_CXX_STANDARD 17)

# -----------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arms_msgs REQUIRED)

# -----------------------------------------------------------------------
# Library: state machine + PID
# -----------------------------------------------------------------------
add_library(arms_control_lib
  src/pid_controller.cpp
  src/state_machine.cpp
  src/crsf_output.cpp
  src/servo_lock.cpp
)

target_include_directories(arms_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arms_control_lib
  rclcpp
  arms_msgs
)

# -----------------------------------------------------------------------
# Executable: arms_control_node
# -----------------------------------------------------------------------
add_executable(arms_control_node src/arms_control_node.cpp)

target_link_libraries(arms_control_node
  arms_control_lib
)

ament_target_dependencies(arms_control_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  arms_msgs
)

# -----------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------
install(TARGETS arms_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------
# Python: SITL bridge node (hybrid package)
# -----------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  arms_control/sitl_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sitl_bridge_node
)

ament_package()
