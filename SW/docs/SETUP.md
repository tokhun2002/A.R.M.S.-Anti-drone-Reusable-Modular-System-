# A.R.M.S. 설치 및 실행 매뉴얼

두 가지 모드를 다룹니다.

- **[A] 실기체** — Jetson + FPV 수신기 + FC(Flight Controller)
- **[B] SITL** — PX4 + Gazebo Harmonic (시뮬레이션)

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
# 카메라 드라이버 (실기체 FPV 캡처 — usb_cam 대신 v4l2_camera 사용)
#   아날로그→USB 캡처 동글(MS210x/EasierCAP)은 프레임 간격을 stepwise 로
#   보고해 usb_cam 이 포맷 열거에 실패한다. v4l2_camera(C++)는 정상 동작.
sudo apt install ros-humble-v4l2-camera

# NumPy 1.x 고정 (apt python3-opencv 4.5.4 는 NumPy 1.x 로 빌드됨.
#   pip numpy 2.x 가 있으면 cv2 import 시 크래시)
pip3 install "numpy<2"

# MAVSDK (arms_control C++ 빌드에 필요)
sudo apt install libmavsdk-dev

# GPIO (실 기체 Jetson에서만 필요)
sudo apt install libgpiod-dev

# ros_gz_bridge (SITL에서만 필요 — Harmonic은 B-1 소스 빌드 참고)
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

### A-1. Detection 컨테이너 빌드 (최초 1회)

```bash
cd SW/arms_ws/src/arms_detection/docker

# 모델 파일 배치 — Jetson은 TensorRT .engine
cp /path/to/best.engine models/

# 이미지 빌드 (최초 or Dockerfile 변경 시에만)
docker compose -f docker-compose.jetson.yml build
```

> 컨테이너 기동은 A-2 의 `arms.launch.py` 가 `docker compose up -d` 로 자동 처리합니다.
> `ROS_DOMAIN_ID` 가 기본값(0)이 아니면 런치 실행 전에 `export ROS_DOMAIN_ID=<n>` 하세요.

### A-2. 전체 시스템 시작

```bash
ros2 launch arms_bringup arms.launch.py
```

노드 스택과 함께 detection 컨테이너(`docker compose up -d`)까지 자동 기동됩니다.

## B. SITL 실행

| 항목          | 버전           |
| ------------- | -------------- |
| Gazebo        | Harmonic (8.x) |
| PX4-Autopilot | 최신           |
| Python        | 3.10+          |

### B-1. SITL 환경 구성 (최초 1회)

#### ros_gz_bridge (소스 빌드 필수)

> ⚠️ apt의 `ros-humble-ros-gz-bridge`는 Gazebo Fortress (`ign-msgs 8`, `ign-transport 11`) 기준으로 빌드되어 있습니다. 반면 PX4 SITL은 Gazebo Harmonic (`gz-msgs 10`, `gz-transport 13`)을 사용하므로 버전이 맞지 않아 `Unknown message type [9]` 에러가 발생하고 ROS2 토픽이 브릿지되지 않습니다. 반드시 Harmonic 버전으로 소스 빌드해야 합니다.

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

#### worlds / models 등록

`make px4_sitl`은 빌드만 하며 커스텀 모델/월드는 자동 등록되지 않습니다.
`SW/setup_sim.sh` 가 심볼릭 링크 + airframe 등록 + PX4/ROS 빌드를 한 번에 처리합니다.

```bash
cd SW
./setup_sim.sh                 # PX4 위치가 다르면: PX4_DIR=<경로> ./setup_sim.sh
```

<details>
<summary>수동으로 링크하려면</summary>

```bash
# 월드 파일 링크
cd ~/PX4-Autopilot/Tools/simulation/gz/worlds/
ln -s /path/to/SW/simulation/worlds/arms_sitl.sdf .

# 모델 폴더 링크
cd ~/PX4-Autopilot/Tools/simulation/gz/models/
ln -s /path/to/SW/simulation/models/arms_drone .
```

</details>

### B-2. 실행 — 원클릭 (권장)

```bash
cd SW
source arms_ws/install/setup.bash     # 새 터미널이면 한 번
./run_arms.sh
```

`run_arms.sh` 하나로 socat 가상 시리얼 생성 + PX4 SITL(arms_drone) + Gazebo + 전체 ROS 스택 + 패널 GUI 가 다 뜹니다.
YOLO 검출을 쓰려면 Detection 컨테이너를 따로 실행합니다(B-2′ Terminal 3).

### B-2′. 실행 — 수동 (터미널 3개)

원클릭을 안 쓰고 단계별로 실행하려면:

**Terminal 1 — PX4 SITL + Gazebo**

```bash
cd ~/path/to/~PX4-Autopilot
PX4_GZ_WORLD=arms_sitl make px4_sitl gz_arms_drone
```

Gazebo 창과 PX4 콘솔이 뜨면 정상입니다.

**Terminal 2 — A.R.M.S. 노드**

```bash
source /opt/ros/humble/setup.bash
source /path/to/SW/arms_ws/install/setup.bash
source ~/ros_gz_harmonic_ws/install/setup.bash

ros2 launch arms_bringup arms_sitl.launch.py
```

`arms_sitl.launch.py` 가 gz 카메라 브리지 + arms_control_node + sitl_bridge_node + arms_ui_node + arms_detection_node + 패널 GUI 를 한 번에 기동합니다.

**Terminal 3 — YOLO Detection 컨테이너 (선택)**

```bash
cd /path/to/SW/arms_ws/src/arms_detection/docker

# 노트북
docker compose -f docker-compose.laptop.yml up

# 젯슨
docker compose -f docker-compose.jetson.yml up
```

> YOLO 컨테이너 없이도 HSV/ABSDIFF 검출만으로 독립 동작합니다.

## 동작 확인

### 토픽 모니터링

```bash
# 상태 머신 상태 확인
ros2 topic echo /arms/mission_state

# detection 확인
ros2 topic echo /arms/detections

# 발사 판정용 looming 확인 (x=τ, y=bbox크기, z=팽창률)
ros2 topic echo /arms/debug_looming

# 카메라 영상 확인
ros2 run rqt_image_view rqt_image_view /arms/image_raw
```

### 정상 상태 전이 흐름

```
시작         →  IDLE
arm 완료     →  SEARCH   (드론 호버링, detection 대기)
연속 인식    →  LOCK     (잠금 타이머 → arm)
launch 버튼  →  TRACK    (PID 추적, P 게인 시간 램프)
거리 < 5m    →  FIRE     (페이로드 발사)
발사 완료    →  RTL      (귀환/착륙)
착륙         →  IDLE
```

### 노드 그래프 확인

```bash
ros2 node list
ros2 topic list
rqt_graph
```

### 카메라 테스트

```bash
ffplay -f v4l2 -framerate 30 -video_size 720x480 -i /dev/video0
```
