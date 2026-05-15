# A.R.M.S. SW

## 문서

| 문서                                | 설명                                                                                                |
| ----------------------------------- | --------------------------------------------------------------------------------------------------- |
| [DESIGN.md](docs/DESIGN.md)         | 소프트웨어 아키텍처 설계 문서. 노드 구조, 상태 머신, PID 제어, MAVLink 인터페이스 등 전체 설계 기술 |
| [SITL_SETUP.md](docs/SITL_SETUP.md) | PX4 + Gazebo Harmonic 기반 SITL 환경 구성 및 실행 가이드                                            |
| [RUN.md](docs/RUN.md)               | 실기체 실행 가이드                                                                                  |
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
├── simulation/         # Gazebo 월드 및 모델 파일
├── YOLO_balloon/       # YOLOv11 학습 및 ONNX 모델
└── gpio_ws/            # GPIO 테스트 워크스페이스
```
