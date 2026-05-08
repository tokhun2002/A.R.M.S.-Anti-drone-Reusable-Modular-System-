# A.R.M.S. SITL Setup Guide

PX4 + Gazebo Harmonic 기반 SITL 환경 구성 매뉴얼.

---

## 1. Prerequisites

| 항목 | 버전 | 설치 확인 |
|------|------|-----------|
| Ubuntu | 22.04 LTS | `lsb_release -a` |
| ROS2 | Humble | `ros2 --version` |
| Gazebo | Harmonic (8.x) | `gz sim --version` |
| PX4-Autopilot | v1.14+ | `make --version` |
| Python | 3.10+ | `python3 --version` |
| pymavlink | latest | `pip show pymavlink` |

---

## 2. PX4-Autopilot 설치

```bash
# 소스 클론
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 의존성 설치 (Ubuntu 22.04)
bash ./Tools/setup/ubuntu.sh

# Gazebo Harmonic 플러그인 설치
pip install --user pyros-genmsg
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
```

---

## 3. Gazebo Harmonic 설치

```bash
# Gazebo Harmonic 공식 설치
sudo apt-get update
sudo apt-get install gz-harmonic

# 설치 확인
gz sim --version
```

---

## 4. arms_drone 모델 등록

Gazebo가 커스텀 모델을 찾을 수 있도록 환경변수를 설정한다.

```bash
# simulation/models 디렉토리를 Gazebo 모델 경로에 추가
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/SW/simulation/models

# 영구 설정 (bashrc에 추가)
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/SW/simulation/models' >> ~/.bashrc
source ~/.bashrc
```

> `/path/to/SW/simulation/models` 를 실제 절대 경로로 교체하세요.

---

## 5. PX4 x500 베이스 모델 확인

`simulation/models/arms_drone/model.sdf` 는 `x500` 모델을 `<include>` 로 참조한다.
PX4-Autopilot 모델 경로도 추가해야 한다.

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
/path/to/PX4-Autopilot/Tools/simulation/gz/models
```

---

## 6. ROS2 워크스페이스 빌드

```bash
cd /path/to/SW/arms_ws
source /opt/ros/humble/setup.bash

# 빌드
colcon build --symlink-install

# 환경 소싱
source install/setup.bash
```

---

## 7. SITL 실행 순서

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd /path/to/PX4-Autopilot

# arms_sitl.sdf 월드로 x500 SITL 실행
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_WORLD=/path/to/SW/simulation/worlds/arms_sitl.sdf \
PX4_GZ_MODEL=arms_drone \
./build/px4_sitl_default/bin/px4 -s etc/init.d-posix/rcS
```

> 월드 파일 경로는 절대 경로로 지정해야 한다.

### Terminal 2 — ros_gz_bridge (Gazebo ↔ ROS2)

```bash
source /opt/ros/humble/setup.bash
source /path/to/SW/arms_ws/install/setup.bash

ros2 launch /path/to/SW/simulation/launch/bridge.launch.py
```

### Terminal 3 — A.R.M.S. 노드

```bash
source /opt/ros/humble/setup.bash
source /path/to/SW/arms_ws/install/setup.bash

ros2 launch arms_bringup arms_sitl.launch.py
```

---

## 8. 동작 확인

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
4. `lock_elapsed_sec >= 2.0` → (실제 환경: launch 버튼 누름) SITL에서는 수동으로 아래 명령 실행:

```bash
# LOCK 상태에서 TRACK으로 강제 전이 테스트 (SITL 전용)
# arms_control_node의 on_launch_button()을 직접 호출하는 ROS2 서비스 추가 예정
```

5. 거리 < `fire_distance_m` (5.0 m) → `FIRE` → `RTL`

---

## 9. 빨간 공 위치 변경

`simulation/worlds/arms_sitl.sdf` 에서 `red_ball` 모델의 `<pose>` 를 수정한다.

```xml
<!-- 형식: x y z roll pitch yaw -->
<pose>5 3 10 0 0 0</pose>
```

예시:
- 정면 위 10m: `<pose>0 0 10 0 0 0</pose>`
- 왼쪽 45도 위: `<pose>-5 0 5 0 0 0</pose>`
- 멀리: `<pose>10 10 15 0 0 0</pose>`

---

## 10. MAVLink 포트 설정

PX4 SITL 기본 MAVLink 포트:

| 용도 | 주소 |
|------|------|
| GCS (QGroundControl) | UDP 14550 |
| Offboard API | UDP 14540 |
| arms_control_node | UDP 14550 (기본값) |

`control_params.yaml` 의 `mavlink.connection` 을 `"udp:127.0.0.1:14540"` 으로 변경하면 QGC와 동시 연결 가능.

---

## 11. 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `Heartbeat timeout` | PX4 SITL 미실행 또는 포트 불일치 | Terminal 1 상태 확인, 포트 확인 |
| `/arms/image_raw` 없음 | bridge 미실행 또는 모델 토픽 경로 불일치 | Terminal 2 확인, `gz topic -l` 로 Gazebo 토픽 확인 |
| `x500` 모델 없음 | `GZ_SIM_RESOURCE_PATH` 미설정 | Section 5 참조 |
| `OFFBOARD rejected` | 스트리밍 시작 전 모드 변경 시도 | `start_offboard_stream()` 후 2초 대기 확인 |
| 상태가 SEARCH에서 멈춤 | detection 노드 미실행 또는 YOLO 서버 미기동 | `/arms/detections` 토픽 echo 확인 |
