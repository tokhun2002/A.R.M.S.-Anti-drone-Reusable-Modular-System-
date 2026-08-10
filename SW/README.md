# A.R.M.S. SW

비전 기반 자율 요격 드론 SITL 시뮬레이션 (ROS2 Humble + PX4 SITL + Gazebo Harmonic).

## 빠른 시작

### 환경

| 항목       | 내용                                      |
| ---------- | ----------------------------------------- |
| OS         | Ubuntu 22.04                              |
| ROS        | ROS2 Humble                               |
| 시뮬레이터 | Gazebo Harmonic                           |
| SDK        | MAVSDK 설치됨                             |
| PX4        | `~/PX4-Autopilot` 에 clone 되어 있어야 함 |

### 설치

```bash
# 1) 최초 한 번 — 모델 링크 + airframe 등록 + PX4 빌드 + ROS 빌드
cd SW
./setup_sim.sh                       # PX4 다른 위치면: PX4_DIR=<경로> ./setup_sim.sh

# 2) 실행 — 매번 이것만
source arms_ws/install/setup.bash    # 새 터미널이면 한 번
./run_arms.sh
```

`setup_sim.sh` 가 드론 모델(arms_drone, 커스텀 x500_base 형태/무게)을 PX4 모델 폴더에
자동 심볼릭 링크하고, airframe `4021_gz_arms_drone` 등록 + PX4/ROS 빌드까지 처리한다.
이후 `run_arms.sh` 하나로 PX4 SITL + Gazebo + 전체 ROS 스택 + 패널 GUI 가 다 뜬다.

### 미션 흐름

```
IDLE → (auto-arm) → SEARCH(지상 대기) → 풍선 인식 → LOCK
     → [패널 LAUNCH] → TRACK(추적, P 게인 시간 램프) → 거리<5m → FIRE → RTL
```

### 패널 조작

- **🚀 LAUNCH** : LOCK 에서 누르면 TRACK 시작
- **시작 P / 최대 P / P 증가 시간** : TRACK 진입 후 시작 P 로 출발 → 설정 시간 동안 최대 P 까지 서서히 증가
  (시작부터 센 P 면 중심을 잃어서 천천히 올리는 구조)
- **최대 각도 [deg]** : 자세 명령 한계각 (요격용 기본 90, 실사용 80~85 권장)
- **Roll/Pitch sign** : 드론이 풍선 반대로 가면 부호 뒤집기
- **풍선 비행/고도** : 표적 풍선 제어 (기본 시작 고도 50m)

### 안 될 때

- SEARCH 에서 안 넘어감 → 검출 안 됨. 카메라 창에 풍선 보이는지, 검출 모드(YOLO ON/ABSDIFF/BOTH) 바꿔보기
- 드론이 풍선 반대로 감 → Roll/Pitch sign 뒤집기
- 추적 느림 → 최대 P ↑ / P 증가 시간 ↓ ; 근접 발작 → 최대 P ↓ / 증가 시간 ↑
- `arms_drone` 모델 못 찾음 → `setup_sim.sh` 다시 실행(링크 갱신) 또는 `run_arms.sh` 의 `GZ_SIM_RESOURCE_PATH` 확인

## 문서

| 문서                                | 설명                                                                                                |
| ----------------------------------- | --------------------------------------------------------------------------------------------------- |
| [DESIGN.md](docs/DESIGN.md)         | 소프트웨어 아키텍처 설계 문서. 노드 구조, 상태 머신, PID 제어, MAVLink 인터페이스 등 전체 설계 기술 |
| [SETUP.md](docs/SETUP.md)           | 설치 및 실행 매뉴얼 (실기체 + SITL 환경 구성/실행 통합)                                             |
| [HOW_TO_GIT.md](docs/HOW_TO_GIT.md) | Git/GitHub 팀 사용 가이드. 기본 개념, 자주 쓰는 명령어, 작업 시나리오                               |
| [TODO.md](docs/TODO.md)             | 개발 진행 현황 및 테스트 체크리스트                                                                 |

## 폴더 구조

```
SW/
├── docs/               # 문서
├── arms_ws/            # ROS2 워크스페이스
│   └── src/
│       ├── arms_bringup/       # 런치 파일
│       ├── arms_video/         # 영상 수신
│       ├── arms_detection/     # YOLO 객체 인식 (Docker)
│       ├── arms_control/       # 상태 머신 + PID + MAVLink
│       ├── arms_ui/            # 오퍼레이터 UI
│       └── arms_msgs/          # 커스텀 메시지
├── simulation/         # Gazebo 월드, 모델(arms_drone/x500/x500_base), airframes(4021)
├── tools/              # balloon_referee_diagonal(런치 자동 실행) / log_detections / kf_analyze
├── setup_sim.sh        # 최초 셋업 (한 번)
├── run_arms.sh         # 실행 (매번)
├── YOLO_balloon/       # YOLOv11 학습
└── gpio_ws/            # GPIO 테스트 워크스페이스
```
