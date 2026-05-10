# A.R.M.S. SITL Setup Guide

PX4 + Gazebo Harmonic 기반 ARMS 전용 SITL 환경 구성 매뉴얼

## 1. Prerequisites

| 항목          | 버전           | 설치 확인           |
| ------------- | -------------- | ------------------- |
| Ubuntu        | 22.04 LTS      | `lsb_release -a`    |
| ROS2          | Humble         | `ros2 --version`    |
| Gazebo        | Harmonic (8.x) | `gz sim --version`  |
| PX4-Autopilot | v1.14+         | `make --version`    |
| Python        | 3.10+          | `python3 --version` |

## 2. ROS2 워크스페이스 빌드

- MAVSDK 및 관련 의존성 설치 필요할 수 있음

```bash
cd /path/to/SW/arms_ws
source /opt/ros/humble/setup.bash

# 빌드
colcon build --symlink-install

# 환경 소싱
source install/setup.bash
```

## 3. worlds, models 파일 등록

- `make px4_sitl`은 빌드만 하며, 커스텀 모델/월드 파일은 자동으로 등록되지 않는다.
- `SW/simulation`에 있는 worlds와 models을 PX4에서 불러올 수 있게 심볼릭 링크를 만들어줘야 한다.

```bash
# 월드 파일 링크
cd /path/to/PX4-Autopilot/Tools/simulation/gz/worlds/
ln -s /path/to/SW/simulation/worlds/arms_sitl.sdf .

# 모델 폴더 링크
cd /path/to/PX4-Autopilot/Tools/simulation/gz/models/
ln -s /path/to/SW/simulation/models/arms_drone .
```

## 4. SITL 실행

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd /path/to/PX4-Autopilot

PX4_GZ_WORLD=arms_sitl \
PX4_GZ_MODEL=arms_drone \
make px4_sitl gz_x500
```

### Terminal 2 — A.R.M.S. 노드

```bash
source /opt/ros/humble/setup.bash
source /path/to/SW/arms_ws/install/setup.bash

ros2 launch arms_bringup arms_sitl.launch.py
```

`arms_sitl.launch.py` 가 내부적으로 아래를 한 번에 기동한다:

- `gz_ros2_bridge` — Gazebo 카메라/ray sensor → `/arms/image_raw`, `/arms/scan_raw`
- `arms_control_node` — 상태머신 + PID + MAVLink(UDP)
- `arms_ui_node` — OpenCV 오버레이

## 5. 동작 확인

```bash
# 상태 머신 확인
ros2 topic echo /arms/mission_state

# 카메라 영상 확인 (별도 터미널)
ros2 run rqt_image_view rqt_image_view /arms/image_raw

# 거리 센서 확인
ros2 topic echo /arms/scan_raw

# 토픽 목록
ros2 topic list
```

정상 동작 시 시퀀스:

1. `mission_state.state = IDLE`
2. 약 2초 후 auto-arm → `SEARCH`
3. 카메라에 빨간 공이 인식되면 → `LOCK`
4. launch 버튼 입력 → `TRACK`
5. 거리 < `fire_distance_m` (5.0 m) → `FIRE` → `RTL`

## 6. 빨간 공 위치 변경

`simulation/worlds/arms_sitl.sdf` 에서 `red_ball` 모델의 `<pose>` 를 수정한다.

```xml
<!-- 형식: x y z roll pitch yaw -->
<pose>5 3 10 0 0 0</pose>
```

예시:

- 정면 위 10m: `<pose>0 0 10 0 0 0</pose>`
- 왼쪽 45도 위: `<pose>-5 0 5 0 0 0</pose>`
- 멀리: `<pose>10 10 15 0 0 0</pose>`

## 7. MAVLink 포트 설정

PX4 SITL 기본 MAVLink 포트:

| 용도                 | 주소               |
| -------------------- | ------------------ |
| GCS (QGroundControl) | UDP 14550          |
| Offboard API         | UDP 14540          |
| arms_control_node    | UDP 14550 (기본값) |

`control_params.yaml` 의 `mavlink.connection` 을 `"udp:127.0.0.1:14540"` 으로 변경하면 QGC와 동시 연결 가능.

## 8. 트러블슈팅

| 증상                   | 원인                                                    | 해결                                               |
| ---------------------- | ------------------------------------------------------- | -------------------------------------------------- |
| `Heartbeat timeout`    | PX4 SITL 미실행 또는 포트 불일치                        | Terminal 1 상태 확인, 포트 확인                    |
| `/arms/image_raw` 없음 | `arms_sitl.launch.py` 미실행 또는 모델 토픽 경로 불일치 | Terminal 2 확인, `gz topic -l` 로 Gazebo 토픽 확인 |
| `OFFBOARD rejected`    | 스트리밍 시작 전 모드 변경 시도                         | `start_offboard_stream()` 후 2초 대기 확인         |
| 상태가 SEARCH에서 멈춤 | detection 컨테이너 미기동                               | `docker ps`, `/arms/detections` 토픽 echo 확인     |
