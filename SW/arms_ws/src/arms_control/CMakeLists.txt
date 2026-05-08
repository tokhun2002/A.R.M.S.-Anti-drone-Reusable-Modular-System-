cmake_minimum_required(VERSION 3.14)
project(arms_control)

set(CMAKE_CXX_STANDARD 17)

# -----------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arms_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

# Optional: Jetson GPIO via libgpiod
find_library(GPIOD_LIB gpiod)

# -----------------------------------------------------------------------
# Library: state machine + PID + MAVLink interface
# -----------------------------------------------------------------------
add_library(arms_control_lib
  src/pid_controller.cpp
  src/state_machine.cpp
  src/mavlink_interface.cpp
)

target_include_directories(arms_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(arms_control_lib
  MAVSDK::mavsdk
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
  arms_msgs
)

# GPIO support (optional)
if(GPIOD_LIB)
  target_compile_definitions(arms_control_node PRIVATE HAS_GPIO)
  target_link_libraries(arms_control_node ${GPIOD_LIB})
  message(STATUS "libgpiod found — GPIO support enabled.")
else()
  message(STATUS "libgpiod not found — GPIO disabled.")
endif()

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

ament_package()
