cmake_minimum_required(VERSION 3.8)
project(arms_video)

find_package(ament_cmake REQUIRED)

# 런치/설정 파일만 설치하는 순수 launch 패키지.
# 실기체 캡처는 v4l2_camera(C++), 샘플 replay 는 image_publisher(C++) 를
# 런치에서 구동한다.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
