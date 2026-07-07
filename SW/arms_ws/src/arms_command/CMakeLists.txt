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

# Optional GPIO support
find_library(GPIOD_LIB gpiod)

add_library(arms_command_hw STATIC
  src/ads1115.cpp
  src/gpio_input.cpp
)
target_include_directories(arms_command_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(GPIOD_LIB)
  target_compile_definitions(arms_command_hw PRIVATE HAS_GPIOD)
  target_link_libraries(arms_command_hw ${GPIOD_LIB})
  message(STATUS "libgpiod found — GPIO support enabled.")
else()
  message(STATUS "libgpiod not found — GPIO real-input support disabled. fake_mode still works.")
endif()

add_executable(controller_input_node src/controller_input_node.cpp)
target_include_directories(controller_input_node PRIVATE include)
target_link_libraries(controller_input_node arms_command_hw)
ament_target_dependencies(controller_input_node rclcpp sensor_msgs)

install(TARGETS controller_input_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
