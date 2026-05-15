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

## 3. ROS2 워크스페이스 빌드

- MAVSDK 및 관련 의존성 설치 필요할 수 있음

```bash
cd /path/to/SW/arms_ws
source /opt/ros/humble/setup.bash

# 빌드
colcon build --symlink-install

# 환경 소싱
source install/setup.bash
```

## 4. worlds, models 파일 등록

- `make px4_sitl`은 빌드만 하며, 커스텀 모델/월드 파일은 자동으로 등록되지 않음.
- `SW/simulation`에 있는 worlds와 models을 PX4에서 불러올 수 있게 심볼릭 링크를 만들어야 함.

```bash
# 월드 파일 링크
cd /path/to/PX4-Autopilot/Tools/simulation/gz/worlds/
ln -s /path/to/SW/simulation/worlds/arms_sitl.sdf .

# 모델 폴더 링크
cd /path/to/PX4-Autopilot/Tools/simulation/gz/models/
ln -s /path/to/SW/simulation/models/arms_drone .
```

## 5. SITL 실행

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd /path/to/PX4-Autopilot

PX4_GZ_WORLD=arms_sitl \
PX4_GZ_MODEL=arms_drone \
make px4_sitl gz_x500
```

- `SW/simulation`에 만들어둔 world, model로 PX4 SITL을 실행

### Terminal 2 — A.R.M.S. 노드

```bash
source /opt/ros/humble/setup.bash
source /path/to/SW/arms_ws/install/setup.bash
source /path/to/ros_gz_harmonic_ws/install/setup.bash

ros2 launch arms_bringup arms_sitl.launch.py
```

`arms_sitl.launch.py` 가 내부적으로 아래를 한 번에 기동:

- `gz_ros2_bridge` — Gazebo 카메라/ray sensor → `/arms/image_raw`, `/arms/scan_raw`
- `arms_control_node` — 상태머신 + PID + MAVLink(UDP)
- `arms_ui_node` — OpenCV 오버레이

## 6. 동작 확인

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

## 7. 빨간 공 위치 변경

`simulation/worlds/arms_sitl.sdf` 에서 `red_ball` 모델의 `<pose>` 를 수정한다.

```xml
<!-- 형식: x y z roll pitch yaw -->
<pose>5 3 10 0 0 0</pose>
```

예시:

- 정면 위 10m: `<pose>0 0 10 0 0 0</pose>`
- 왼쪽 45도 위: `<pose>-5 0 5 0 0 0</pose>`
- 멀리: `<pose>10 10 15 0 0 0</pose>`

## 8. MAVLink 포트 설정

PX4 SITL 기본 MAVLink 포트:

| 용도                 | 주소               |
| -------------------- | ------------------ |
| GCS (QGroundControl) | UDP 14550          |
| Offboard API         | UDP 14540          |
| arms_control_node    | UDP 14550 (기본값) |

`control_params.yaml` 의 `mavlink.connection` 을 `"udp:127.0.0.1:14540"` 으로 변경하면 QGC와 동시 연결 가능.

## 9. 트러블슈팅

| 증상                                | 원인                                                    | 해결                                                                             |
| ----------------------------------- | ------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `Heartbeat timeout`                 | PX4 SITL 미실행 또는 포트 불일치                        | Terminal 1 상태 확인, 포트 확인                                                  |
| `/arms/image_raw` 없음              | `arms_sitl.launch.py` 미실행 또는 모델 토픽 경로 불일치 | Terminal 2 확인, `gz topic -l` 로 Gazebo 토픽 확인                               |
| `Unknown message type [9]`          | ros_gz_bridge가 Fortress용으로 빌드되어 Harmonic 불일치 | 섹션 2.3 소스 빌드 절차 수행 후 `source ~/ros_gz_harmonic_ws/install/setup.bash` |
| `OFFBOARD rejected`                 | 스트리밍 시작 전 모드 변경 시도                         | `start_offboard_stream()` 후 2초 대기 확인                                       |
| 상태가 SEARCH에서 멈춤              | detection 컨테이너 미기동                               | `docker ps`, `/arms/detections` 토픽 echo 확인                                   |
| Accel/Gyro/Baro/Compass sensor 없음 | arms_sitl.sdf world에 센서 시뮬레이션 플러그인 누락     | world 파일에 Imu/AirPressure/Magnetometer/NavSat 플러그인 확인                   |
