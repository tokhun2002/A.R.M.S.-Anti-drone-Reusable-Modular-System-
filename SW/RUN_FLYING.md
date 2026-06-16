# A.R.M.S. SITL — 날아다니는 풍선 요격 데모 실행 가이드

기존 `SITL_SETUP.md` 의 1~3 단계(워크스페이스 빌드, world/model 심볼릭 링크)는
그대로 따른 상태라고 가정한다. 이 문서는 **풍선이 날아다니고 그걸 잡아서 FIRE 까지
가는** SITL 데모를 굴리는 순서다.

## 0. 바뀐/추가된 것 요약

| 파일 | 변경 |
| --- | --- |
| `simulation/worlds/arms_sitl.sdf` | `red_ball` 을 non-static + 중력off 로 바꿔서 referee 가 움직일 수 있게 함 |
| `arms_control_node.cpp` / `control_params.yaml` | ① `/arms/launch_cmd` 토픽으로 launch 버튼 대체 ② `sitl_auto_launch` 로 LOCK→TRACK 자동 전환 ③ `track_throttle` 로 TRACK 시 상승해서 거리 좁힘 ④ `roll_sign`/`pitch_sign` 으로 재빌드 없이 제어 방향 뒤집기 |
| `tools/redball_detector.py` | **torch/CUDA/Docker 없이** OpenCV 로 빨간 공 검출 → `/arms/detections`. SITL 검출 단계를 바로 뚫는 용도 |
| `tools/balloon_referee.py` | 풍선을 하늘에서 비행시키고, FIRE 감지 시 풍선 제거(명중 연출) |

## 1. 재빌드 (C++ 바뀜)

```bash
cd <SW>/arms_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arms_control
source install/setup.bash
```

월드 파일을 바꿨으니 심볼릭 링크가 새 파일을 가리키는지 확인(원래 링크면 그대로 OK).

## 2. 실행 (터미널 5개)

### T1 — PX4 SITL + Gazebo
```bash
cd <PX4-Autopilot>
PX4_GZ_WORLD=arms_sitl PX4_GZ_MODEL=arms_drone make px4_sitl gz_x500
```

### T2 — ARMS 노드 (브리지 + 제어 + UI)
```bash
source /opt/ros/humble/setup.bash
source <SW>/arms_ws/install/setup.bash
ros2 launch arms_bringup arms_sitl.launch.py
```

### T3 — 검출기 (YOLO 대신 OpenCV)
```bash
source /opt/ros/humble/setup.bash
source <SW>/arms_ws/install/setup.bash
python3 <SW>/tools/redball_detector.py
```

### T4 — 풍선 비행 + 명중 심판
```bash
source /opt/ros/humble/setup.bash
source <SW>/arms_ws/install/setup.bash
python3 <SW>/tools/balloon_referee.py
```

### T5 — 상태 모니터
```bash
ros2 topic echo /arms/mission_state
```

## 3. 기대 시퀀스 (자동)

```
IDLE → (2s, auto-arm) → SEARCH → 빨간 공 인식(2s) → LOCK
     → (1s, auto-launch) → TRACK → 상승해서 거리<5m → FIRE → RTL
                                                  └ referee 가 풍선 제거 + "🎯 명중!"
```

`sitl_auto_launch: false` 로 두면 T4 대신 수동으로:
```bash
ros2 topic pub --once /arms/launch_cmd std_msgs/msg/Empty {}
```

## 4. 안 될 때 — 가장 흔한 순서대로

1. **SEARCH 에서 안 넘어감** → 검출 안 됨.
   `ros2 topic echo /arms/detections` 비어있으면 카메라에 빨간 공이 안 잡히는 것.
   UI 창에서 공이 화면에 보이는지, HSV 범위(`redball_detector.py` 상단) 조정.
2. **드론이 타겟에서 멀어짐** → 제어 부호 반대.
   `control_params.yaml` 의 `roll_sign` / `pitch_sign` 을 `-1.0` 으로 바꿔가며 시도(재빌드 X, 노드만 재시작).
3. **TRACK 인데 거리가 안 줄어듦** → 상승 부족.
   `track_throttle` 를 0.62 → 0.68 식으로 올림. 너무 크면 타겟 지나쳐 올라가니 주의.
4. **거리<5 인데 FIRE 안 됨** → ray 센서가 공을 못 맞춤(공이 ray 정면 밖).
   `red_ball` 비행 반경 `R`(referee) 을 줄이거나 `fire_distance_m` 를 키움.
5. **`gz service` 에러(풍선이 안 움직임)** → 월드 이름/모델 이름 확인.
   `gz model -l` 로 `red_ball` 있는지, 월드가 `arms_sitl` 인지 확인.

## ⚠️ 솔직한 한계

- 이 코드는 내 환경에서 **컴파일/실행 검증을 못 했다** (ROS2·Gazebo·PX4·MAVSDK 없음).
  문법/논리는 맞춰놨지만 첫 빌드에서 사소한 에러는 나올 수 있음.
- "명중" 은 **실제 그물/투사체 물리 시뮬이 아니라** FIRE 상태 도달 시 풍선을 치우는 연출이다.
  실제 충돌·그물 발사 물리까지 가려면 별도 작업 필요.
- PID·throttle 튜닝은 직접 돌려보며 맞춰야 한다. 위 4번 항목이 그 가이드.
