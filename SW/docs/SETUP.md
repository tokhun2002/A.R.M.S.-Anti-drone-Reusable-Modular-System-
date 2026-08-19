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

### A-3. 샘플 영상 replay (카메라·FC 없이 파이프라인 테스트)

저장된 샘플 영상으로 detection → control → UI 파이프라인을 검증합니다. 캡처 동글이나
FC 없이 검출·추적·UI를 그대로 돌려볼 수 있어, 검출 튜닝이나 UI 확인에 유용합니다.
라이브 카메라 대신 `image_publisher` 가 영상 파일을 `/arms/image_raw` 로 스트리밍합니다.

detection 은 `arms.launch.py` 와 동일하게 **GPU 도커 컨테이너로 자동 기동**됩니다
(`docker compose up -d`, 멱등). 즉 실기체와 같은 YOLO 검출을 샘플 영상에 그대로 적용합니다.
조종기는 실기체 ESP32 대신 **SITL 가상 조종기(tkinter GUI)** 가 함께 떠서, 그 창에서
arm / 모드 / kill / launch 를 클릭으로 넣어 상태 전이(SEARCH→LOCK→…)를 테스트할 수 있습니다.

```bash
# 기본 샘플(sample1.mov) + 기본 모델(balloon_camera.pt)
ros2 launch arms_bringup arms_replay.launch.py

# 다른 영상 지정 (절대경로 권장)
ros2 launch arms_bringup arms_replay.launch.py \
  video_path:=/path/to/SW/sample_viedo/sample2.mov

# 발행 fps 조정
ros2 launch arms_bringup arms_replay.launch.py publish_rate:=15.0

# 다른 가중치 사용 / detection 컨테이너 끄기
ros2 launch arms_bringup arms_replay.launch.py model:=/models/balloon.engine
ros2 launch arms_bringup arms_replay.launch.py start_detection:=false
```

| 인자              | 기본값                            | 설명                                      |
| ----------------- | --------------------------------- | ----------------------------------------- |
| `video_path`      | `SW/sample_viedo/sample1.mov`     | 재생할 영상 파일 (절대경로 권장)          |
| `publish_rate`    | `30.0`                            | `/arms/image_raw` 발행 fps                |
| `model`           | `/models/balloon_camera.engine`   | detection 컨테이너가 로드할 가중치        |
| `start_detection` | `true`                            | detection 도커 컨테이너 자동 기동 여부    |
| `crsf_port`       | `/dev/ttyUSB0`                    | ELRS TX 시리얼 포트 (제어 출력용)         |
| `fullscreen`      | `false` (replay) / `true` (실기체)| UI 전체화면 여부 (A-4)                    |
| `debug`           | `true` (replay) / `false` (실기체)| 메인 화면 디버그 오버레이 (A-4)           |
| `cv_debug`        | `false`                           | 검출 진단 패널(HSV+그래프) (A-4)          |

샘플 영상은 `SW/sample_viedo/` 에 `sample1.mov`, `sample2.mov`, `sample3.mov` 가 있습니다.

의존성 (영상 스트리밍):

```bash
sudo apt install ros-humble-image-publisher
```

> detection 컨테이너 이미지가 최초라면 A-1 처럼 먼저 `docker compose -f
docker-compose.jetson.yml build` 가 필요합니다.

### A-4. UI 표시 옵션 (`debug` / `cv_debug` / `fullscreen`)

UI 화면 구성은 세 개의 독립 런치 인자로 제어합니다. 실기체(`arms.launch.py`)와
replay(`arms_replay.launch.py`)의 **기본값이 다릅니다**.

| 인자         | 대상 파라미터   | 실기체 기본 | replay 기본 | 켜면 표시되는 것 |
| ------------ | --------------- | ----------- | ----------- | ---------------- |
| `fullscreen` | `ui.fullscreen` | `true`      | `false`     | 전체화면(작업표시줄까지 가림) vs 창모드 |
| `debug`      | `ui.debug`      | `false`     | `true`      | **메인 화면 디버그 오버레이** — err/cmd 숫자 텍스트, YOLO/HSV 상태 텍스트, 오차(ERR)·명령(CMD) 화살표 |
| `cv_debug`   | `ui.cv_debug`   | `false`     | `false`     | **검출 진단 패널** — 화면 오른쪽에 HSV 후보 미리보기 + 실시간 성능 그래프 |

- **`debug` (메인 오버레이).** `false` 면 실기체처럼 메인 화면만 깔끔하게 나옵니다.
  단, **표적 bounding box·confidence 는 `debug` 와 무관하게 항상** 그려집니다(상태 라벨/십자선/
  배터리/락 진행바/전원 버튼도 항상). `true` 면 그 위에 조준 오차·제어 명령 화살표와
  숫자 디버그 텍스트, 검출기(YOLO/HSV) 작동 상태가 추가로 뜹니다.
- **`cv_debug` (진단 패널).** 켜면 UI 가 `/arms/hsv_debug_image`(≈500KB raw)를 구독하고,
  그때만 detection 노드가 그 영상을 발행합니다 → **컨테이너→호스트 UDP 대용량 전송으로
  검출 발행률이 크게 떨어집니다**(측정상 ~30Hz → ~12Hz). 그래서 기본 off 이며, 검출 튜닝·
  성능 확인이 필요할 때만 켭니다. `debug` 와 독립이라 따로 켤 수 있습니다.

```bash
# 실기체를 디버그 오버레이까지 켜서 실행 (기본은 메인 화면만)
ros2 launch arms_bringup arms.launch.py debug:=true

# replay 를 실기체처럼 메인 화면만으로
ros2 launch arms_bringup arms_replay.launch.py debug:=false

# replay 에 검출 진단 패널까지 (검출 발행률 하락 감수)
ros2 launch arms_bringup arms_replay.launch.py cv_debug:=true

# 창모드 replay 를 전체화면으로
ros2 launch arms_bringup arms_replay.launch.py fullscreen:=true
```

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

### 검출 화면 캡처 (`S` 키)

A.R.M.S. UI 창이 떠 있을 때 **`S`(또는 `s`) 키**를 누르면 현재 화면을 JPG로 저장한다.
검출·HSV 튜닝 자료 수집용. **한 번에 두 장**이 같은 타임스탬프 파일명으로 저장된다:

- `SW/output_image/detection/<YYYYmmdd_HHMMSS_mmm>.jpg` — 오버레이 포함 검출 화면
- `SW/output_image/hsv/<...같은 이름>.jpg` — HSV 마스크·후보 화면

- 저장 폴더는 `ui.capture_dir` 파라미터로 바꿀 수 있다(기본 `SW/output_image`).
- 평소 UI 는 `hsv_debug_image` 를 구독하지 않는다(검출 발행률 보호). `S` 를 누른 **그 순간에만
  잠깐 구독**해 HSV 한 프레임을 받아 저장하고 바로 해제하므로, 저장까지 아주 짧은 지연이 있을 수 있다.
  (`cv_debug` 패널이 이미 켜져 있으면 즉시 저장.)
- **A.R.M.S. 창이 포커스**돼 있어야 키가 먹는다(OpenCV `waitKey` 기반).
- `SW/output_image/` 는 런타임 생성물이라 git 추적 대상이 아니다(`.gitignore`).

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

### PlotJuggler 실시간 그래프 (제어 튜닝용)

오차(`/arms/mission_state`)·유도 출력(`/arms/control_debug`)·발사 τ(`/arms/debug_looming`)를
실시간 플롯한다. SITL 튜닝 시 오버슈트·발산을 눈으로 본다.

설치:

```bash
sudo apt install ros-humble-plotjuggler-ros
```

레이아웃 지정 실행:

```bash
ros2 run plotjuggler plotjuggler --buffer_size 60 \
  --layout SW/arms_ws/src/arms_bringup/config/sitl_debug.xml
```

> - 창이 뜨면 상단 **Streaming → Start** → **ROS2 Topic Subscriber** 로 스트리밍을 켜야
>   데이터가 흐른다(토픽 `control_debug`/`debug_looming`/`mission_state`는 레이아웃에 이미 선택됨).
> - 데이터는 `arms_control_node` 가 돌고 있을 때만 나온다(그 노드가 발행). 스택 없이
>   PlotJuggler만 켜면 빈 화면.
