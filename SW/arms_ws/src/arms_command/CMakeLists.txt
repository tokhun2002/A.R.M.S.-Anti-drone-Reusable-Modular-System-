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

# ros2 run 진입점 — 소스가 아니라 scripts/ 의 얇은 런처를 깐다.
#   ★ 예전에는 arms_command/arms_command_node.py 를 RENAME 으로 복사했는데,
#     install(PROGRAMS) 는 colcon --symlink-install 로도 심링크가 되지 않는다.
#     그래서 심링크 빌드를 해도 이 노드만 빌드 시점 코드로 굳어, 소스를 고쳐도
#     조용히 옛 동작을 했다. 런처는 내용이 안 바뀌므로 복사돼도 무해하고,
#     실제 코드는 위 ament_python_install_package 가 깐 모듈에서 온다.
install(PROGRAMS
  scripts/arms_command_node
  DESTINATION lib/${PROJECT_NAME}
)

# ESP32 UART 수신 노드
add_executable(arms_command_hw_node
  src/arms_command_uart_node.cpp
)

ament_target_dependencies(arms_command_hw_node rclcpp sensor_msgs)

install(TARGETS arms_command_hw_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
