cmake_minimum_required(VERSION 3.14)
project(arms_command)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Python 모듈 설치 (import arms_command.xxx 가능하게)
ament_python_install_package(${PROJECT_NAME})

# Python 실행파일을 lib/<pkg>/ 에 직접 설치 (ros2 run이 여기서 찾음)
install(PROGRAMS
  arms_command/arms_command_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arms_command_node
)


# ESP32 UART 수신 노드
add_executable(arms_command_hw_node
  src/arms_command_uart_node.cpp
)

ament_target_dependencies(arms_command_hw_node rclcpp sensor_msgs)

install(TARGETS arms_command_hw_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
