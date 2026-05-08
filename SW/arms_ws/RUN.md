# A.R.M.S. 실행 매뉴얼

두 가지 모드를 다룹니다.

- **[A] 실 기체** — Jetson + FPV 수신기 + FC(Flight Controller)
- **[B] SITL** — PX4 + Gazebo Harmonic (시뮬레이션)

---

## 공통 사전 준비

### OS / ROS2

| 항목   | 버전      |
| ------ | --------- |
| Ubuntu | 22.04 LTS |
| ROS2   | Humble    |

```bash
# ROS2 환경 소싱 (매 터미널마다 또는 bashrc에 추가)
source /opt/ros/humble/setup.bash
```

### 의존성 설치

```bash
# usb_cam
sudo apt install ros-humble-usb-cam

# MAVSDK (arms_control C++ 빌드에 필요)
sudo apt install libmavsdk-dev

# GPIO (실 기체 Jetson에서만 필요)
sudo apt install libgpiod-dev

# ros_gz_bridge (SITL에서만 필요)
sudo apt install ros-humble-ros-gz-bridge

# Docker (detection 컨테이너)
sudo apt install docker.io docker-compose-plugin
sudo usermod -aG docker $USER   # 재로그인 필요
```

### 워크스페이스 빌드

```bash
cd SW/arms_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 이후 모든 터미널에서 반드시 실행:
>
> ```bash
> source /opt/ros/humble/setup.bash
> source SW/arms_ws/install/setup.bash
> ```

## A. 실 기체 실행

### A-1. Detection 컨테이너 시작

```bash
cd SW/arms_ws/src/arms_detection/docker

# 모델 파일 배치 (최초 1회)
cp /path/to/drone.pt models/

# 빌드 후 실행 (최초 or Dockerfile 변경 시)
docker compose up --build

# 이후부터는
docker compose up
```

> `ROS_DOMAIN_ID` 가 기본값(0)이 아닐 경우:
>
> ```bash
> ROS_DOMAIN_ID=1 docker compose up
> ```

### A-2. 전체 시스템 시작

```bash
ros2 launch arms_bringup arms_full.launch.py
```

내부적으로 아래 노드들이 순서대로 기동됩니다:

```
arms_video_node   ← /dev/video0 → /arms/image_raw
arms_control_node ← /arms/detections, GPIO → MAVLink(UART)
arms_ui_node      ← /arms/image_raw, /arms/detections, /arms/mission_state
```

### A-3. 파라미터 오버라이드 (필요 시)

```bash
# MAVLink 포트 변경 예시
ros2 launch arms_bringup arms_full.launch.py \
  mavlink.connection:=/dev/ttyUSB0
```

## B. SITL 실행

> SITL 환경 구성이 처음이라면 `SW/simulation/SITL_SETUP.md` 를 먼저 읽으세요.

터미널 3개를 사용합니다.

### B-1. PX4 SITL + Gazebo 시작

> 최초 실행 전 심볼릭 링크 설정이 필요합니다. `SW/simulation/SITL_SETUP.md` 섹션 3 참고.

```bash
# Terminal 1
cd /path/to/PX4-Autopilot

PX4_GZ_WORLD=arms_sitl \
PX4_GZ_MODEL=arms_drone \
make px4_sitl gz_x500
```

Gazebo 창과 PX4 콘솔이 뜨면 정상입니다.

### B-2. Detection 컨테이너 시작

```bash
# Terminal 2
cd SW/arms_ws/src/arms_detection/docker
docker compose up --build
```

> SITL에서는 실제 드론 모델이 없으므로 detection이 작동하지 않을 수 있습니다.
> 이 경우 아래처럼 테스트용 detection stub을 발행해 LOCK 진입을 테스트할 수 있습니다:
>
> ```bash
> ros2 topic pub /arms/detections arms_msgs/msg/DetectionArray \
>   '{header: {stamp: {sec: 0}}, detections: [{x_center: 0.5, y_center: 0.5, width: 0.1, height: 0.1, confidence: 0.9, class_id: 0, class_name: "drone"}]}' \
>   --rate 30
> ```

### B-3. SITL 시스템 시작 (ros_gz_bridge 포함)

```bash
# Terminal 3
source /opt/ros/humble/setup.bash
source SW/arms_ws/install/setup.bash

ros2 launch arms_bringup arms_sitl.launch.py
```

`arms_sitl.launch.py` 가 내부적으로 아래를 한 번에 기동합니다:

```
gz_ros2_bridge    ← /arms/image_raw, /arms/scan_raw (Gazebo → ROS2)
arms_control_node ← 상태머신 + PID + MAVLink(UDP)
arms_ui_node      ← OpenCV 오버레이
```

정상이면 아래 토픽이 생성됩니다:

```
/arms/image_raw      ← Gazebo 상향 카메라
/arms/scan_raw       ← Gazebo ray sensor (거리)
/arms/mission_state  ← 상태 머신 출력
```

SITL 모드에서는 `arms_control_node` 가 자동으로:

1. MAVLink UDP 연결 (`127.0.0.1:14550`)
2. 2초 대기 후 OFFBOARD 모드 전환
3. Arm → `SEARCH` 상태 진입

## 동작 확인

### 토픽 모니터링

```bash
# 상태 머신 상태 확인
ros2 topic echo /arms/mission_state

# detection 확인
ros2 topic echo /arms/detections

# 카메라 영상 확인
ros2 run rqt_image_view rqt_image_view /arms/image_raw
```

### 정상 상태 전이 흐름

```
시작         →  IDLE
arm 완료     →  SEARCH   (드론 호버링, detection 대기)
2초 연속인식 →  LOCK     (PID 약하게 활성화)
launch 버튼  →  TRACK    (PID 풀 활성화, 추적)
거리 < 5m   →  FIRE     (그물 발사)
발사 완료    →  RTL      (귀환)
착륙         →  IDLE
```

### 노드 그래프 확인

```bash
ros2 node list
ros2 topic list
rqt_graph
```

## 종료

```bash
# ROS2 노드
Ctrl+C  (launch 터미널)

# Detection 컨테이너
docker compose down  (docker 터미널)

# SITL
Ctrl+C  (PX4 터미널)
```

## 자주 발생하는 문제

| 증상                             | 원인                                   | 해결                                                                                   |
| -------------------------------- | -------------------------------------- | -------------------------------------------------------------------------------------- |
| `/arms/image_raw` 토픽 없음      | usb_cam 미설치 또는 `/dev/video0` 없음 | `sudo apt install ros-humble-usb-cam`, `ls /dev/video*` 확인                           |
| `/arms/detections` 토픽 없음     | Docker 컨테이너 미기동                 | `docker compose up` 확인, `docker ps`                                                  |
| Detection이 컨테이너에서 안 보임 | `ROS_DOMAIN_ID` 불일치                 | 호스트와 컨테이너 `ROS_DOMAIN_ID` 동일한지 확인                                        |
| MAVLink heartbeat timeout        | FC 미연결 또는 포트 오류               | 포트/baudrate 확인, `control_params.yaml`의 `mavlink.connection` 확인                  |
| OFFBOARD rejected by PX4         | 스트림 시작 전 모드 전환 시도          | 노드 재시작, 스트리밍 2초 대기 확인                                                    |
| colcon build 실패 (MAVSDK)       | libmavsdk-dev 미설치                   | `sudo apt install libmavsdk-dev`                                                       |
| GPIO 오류 (실 기체)              | libgpiod 미설치 또는 핀 번호 오류      | `sudo apt install libgpiod-dev`, `control_params.yaml`의 `gpio.launch_button_pin` 확인 |
