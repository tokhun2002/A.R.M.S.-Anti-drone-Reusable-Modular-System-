cmake_minimum_required(VERSION 3.8)
project(arms_detection)

find_package(ament_cmake REQUIRED)

# Install docker build context files for reference
install(DIRECTORY docker
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
