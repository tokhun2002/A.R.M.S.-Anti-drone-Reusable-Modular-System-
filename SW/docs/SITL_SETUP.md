# A.R.M.S. SITL Setup Guide

PX4 + Gazebo Harmonic 기반 ARMS 전용 SITL 환경 구성 매뉴얼

## 1. Prerequisites

| 항목          | 버전           |
| ------------- | -------------- |
| Ubuntu        | 22.04 LTS      |
| ROS2          | Humble         |
| Gazebo        | Harmonic (8.x) |
| PX4-Autopilot | 최신           |
| Python        | 3.10+          |

## 2. 의존성 설치

### 2.1 ros_gz_bridge (소스 빌드 필수)

> ⚠️ apt의 `ros-humble-ros-gz-bridge`는 Gazebo Fortress (`ign-msgs 8`, `ign-transport 11`) 기준으로 빌드되어 있음. 반면 PX4 SITL은 Gazebo Harmonic (`gz-msgs 10`, `gz-transport 13`)을 사용하므로 버전이 맞지 않아 `Unknown message type [9]` 에러가 발생하고 ROS2 토픽이 브릿지되지 않음. 반드시 Harmonic 버전으로 소스 빌드해야 함.

```bash
# Harmonic dev 패키지 설치
sudo apt install libgz-msgs10-dev libgz-transport13-dev -y

# 소스 클론 및 빌드
mkdir -p ~/ros_gz_harmonic_ws/src
cd ~/ros_gz_harmonic_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble

cd ~/ros_gz_harmonic_ws
source /opt/ros/humble/setup.bash
GZ_VERSION=harmonic colcon build --packages-select ros_gz_bridge ros_gz_interfaces

source ~/ros_gz_harmonic_ws/install/setup.bash
```

설치 확인:

```bash
ldd ~/ros_gz_harmonic_ws/install/ros_gz_bridge/lib/libros_gz_bridge.so | grep gz-msgs
# libgz-msgs10.so.10 이 보여야 정상
```

## 3. YOLO 모델 가중치 다운로드

YOLO 모델 가중치는 너무 커서 git lfs로 관리됨

### git lfs 설치

```bash
sudo apt install git-lfs
```

### 모델 가중치 다운로드

```bash
git lfs install
git lfs pull
```

## 4. ROS2 워크스페이스 빌드

MAVSDK 및 관련 의존성 설치 필요할 수 있음

```bash
cd /path/to/SW/arms_ws
source /opt/ros/humble/setup.bash

# 빌드
colcon build --symlink-install

# 환경 소싱
source install/setup.bash
```

최초 설치 이후에도 `arms_ws` 코드가 업데이트 된다면 항상 다시 빌드하고 실행해야 함

```bash
# colcon build --packages-select {package name}
colcon build --packages-select arms_control

source install/setup.bash
```

## 5. worlds, models 파일 등록

- `make px4_sitl`은 빌드만 하며, 커스텀 모델/월드 파일은 자동으로 등록되지 않음.
- `SW/simulation`에 있는 worlds와 models을 PX4에서 불러올 수 있게 심볼릭 링크를 만들어야 함.

```bash
# 월드 파일 링크
cd ~/PX4-Autopilot/Tools/simulation/gz/worlds/
ln -s /path/to/SW/simulation/worlds/arms_sitl.sdf .

# 모델 폴더 링크
cd ~/PX4-Autopilot/Tools/simulation/gz/models/
ln -s /path/to/SW/simulation/models/arms_drone .
```

## 6. SITL 실행

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd /path/to/PX4-Autopilot

PX4_GZ_WORLD=arms_sitl \
PX4_GZ_MODEL=arms_drone \
make px4_sitl gz_x500
```

`SW/simulation`에 만들어둔 world, model로 PX4 SITL을 실행

### Terminal 2 — A.R.M.S. 노드

```bash
source /opt/ros/humble/setup.bash
source /path/to/SW/arms_ws/install/setup.bash
source /path/to/ros_gz_harmonic_ws/install/setup.bash

# YOLO detection (docker) 사용 시
ros2 launch arms_bringup arms_sitl.launch.py

# OpenCV 원 감지 사용 시 (docker 불필요, 빠른 검증용)
ros2 launch arms_bringup arms_sitl_opencv.launch.py
```

> 매번 `source ...`하기 귀찮다면 `.bashrc`에 등록하면 됨

### Terminal 3 — YOLO Detection 노드 (arms_sitl.launch.py 사용 시)

```bash
cd /path/to/SW/arms_ws/src/arms_detection/docker

# 노트북
docker compose -f docker-compose.laptop.yml up

# 젯슨
docker compose -f docker-compose.jetson.yml up
```

YOLO 모델을 실행하기 위한 의존성이 모두 설치된 docker container 내부에서 detection node 실행

## 7. 동작 확인

```bash
# 토픽 목록
ros2 topic list

# 카메라 영상 확인
ros2 run rqt_image_view rqt_image_view /arms/image_raw

# 상태 머신 확인
ros2 topic echo /arms/mission_state

# 거리 센서 확인
ros2 topic echo /arms/scan_raw
```

정상 동작 시 시퀀스:

1. `mission_state.state = IDLE`
2. 약 2초 후 auto-arm → `SEARCH`
3. 카메라에 빨간 공이 인식되면 → `LOCK`
4. launch 버튼 입력 → `TRACK`
5. 거리 < `fire_distance_m` (5.0 m) → `FIRE` → `RTL`

## 8. 트러블슈팅

| 증상                                | 원인                                                    | 해결                                                                             |
| ----------------------------------- | ------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `Heartbeat timeout`                 | PX4 SITL 미실행 또는 포트 불일치                        | Terminal 1 상태 확인, 포트 확인                                                  |
| `/arms/image_raw` 없음              | `arms_sitl.launch.py` 미실행 또는 모델 토픽 경로 불일치 | Terminal 2 확인, `gz topic -l` 로 Gazebo 토픽 확인                               |
| `Unknown message type [9]`          | ros_gz_bridge가 Fortress용으로 빌드되어 Harmonic 불일치 | 섹션 2.1 소스 빌드 절차 수행 후 `source ~/ros_gz_harmonic_ws/install/setup.bash` |
| `OFFBOARD rejected`                 | 스트리밍 시작 전 모드 변경 시도                         | `start_offboard_stream()` 후 2초 대기 확인                                       |
| 상태가 SEARCH에서 멈춤              | detection 노드 미기동                                   | `docker ps` 또는 opencv 노드 실행 여부 확인, `/arms/detections` 토픽 echo 확인   |
| Accel/Gyro/Baro/Compass sensor 없음 | arms_sitl.sdf world에 센서 시뮬레이션 플러그인 누락     | world 파일에 Imu/AirPressure/Magnetometer/NavSat 플러그인 확인                   |
