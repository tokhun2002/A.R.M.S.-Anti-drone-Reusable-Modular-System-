# 프로젝트 진행 현황

## Test Checklist

### 노드 작동 확인

| 노드                  | 확인 항목                                                    | SITL | 실기체 |
| --------------------- | ------------------------------------------------------------ | ---- | ------ |
| `arms_video_node`     | 기동 확인, `/arms/image_raw` 발행                            | ■    | ☐      |
| `arms_detection_node` | Docker 컨테이너 기동, `/arms/detections` 발행                | ☐    | ☐      |
| `arms_control_node`   | 기동 확인, 전체 제어 시퀀스 작동, `/arms/mission_state` 발행 | ☐    | ☐      |
| `arms_ui_node`        | 기동 확인, 영상 + 박스 오버레이 표시                         | ☐    | ☐      |

### 상태 전이 확인

| 전이           | 조건                                    | SITL | 실기체 |
| -------------- | --------------------------------------- | ---- | ------ |
| IDLE → SEARCH  | arm 명령 수신                           | ■    | ☐      |
| SEARCH → IDLE  | disarm 명령 수신                        | ☐    | ☐      |
| SEARCH → LOCK  | 연속 감지 >= T_lock (confidence > 0.65) | ☐    | ☐      |
| LOCK → SEARCH  | 타겟 소실 (N 프레임 감지 없음)          | ☐    | ☐      |
| LOCK → TRACK   | launch 버튼 입력 (GPIO)                 | ☐    | ☐      |
| TRACK → SEARCH | 타겟 소실                               | ☐    | ☐      |
| TRACK → FIRE   | 거리 < D_fire (초음파)                  | ☐    | ☐      |
| FIRE → RTL     | 발사 완료                               | ☐    | ☐      |
| RTL → IDLE     | 착륙 / 임무 완료                        | ☐    | ☐      |

## SITL 개발 순서

```
Phase 1: Infra
  [x] ROS workspace + package baseline 구축
  [x] arms_video: Gazebo 카메라 영상 수신 + /arms/image_raw 발행 검증

Phase 2: Perception
  [x] Roboflow Dataset 기반으로 YOLOv11 small 모델 학습
  [ ] arms_detection: Docker로 실행해서 /arms/detections 발행 검증
  [ ] arms_ui: 영상 + 바운딩박스 오버레이 표시 확인

Phase 3: Control
  [ ] arms_control: 상태 머신 구현 및 테스트
  [ ] PID 구현 + MAVLink 연결
  [ ] PID 튜닝
  [ ] launch 버튼 GPIO 입력 대체 (topic?)
  [ ] search ~ fire까지의 전체 과정 테스트
```

## 기타 메모 및 이슈

- docker도 launch 파일에서 실행시켜야 함
- sitl에서 gcs, gps 없이도 이륙 가능하게 설정해야 함
- Phase 2까지 진행 완료되면 jetson 하드웨어에 웹캠 연결해서 같은 전체 과정을 똑같이 진행할 수 있음
- ~~`RUN.md`: 검증 필요. 나중에 `SITL_SETUP.md` 내용을 합쳐야 함~~ → `SETUP.md` 로 통합 완료
