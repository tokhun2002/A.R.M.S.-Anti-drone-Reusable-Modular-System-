# 적용 & 실행 (복붙용)

## 동작 시퀀스 (이번에 바뀜)
```
IDLE → (auto-arm) → SEARCH(프로펠러 OFF, 지상 대기)
     → 풍선 인식 → LOCK(여전히 OFF, 발사 대기)
     → [패널 LAUNCH 버튼] → BOOST(발사각 고정 + 풀스로틀 2초 직진)
                            └ 발사각에서 너무 빗나가면 2초 전에도 즉시 ↓
     → TRACK(픽셀 에러 PID 위치보정) → 거리<5m → FIRE
```
- SEARCH/LOCK 동안 프로펠러 OFF, 땅에 앉아 대기
- LAUNCH 누르면 인식했던 그 각도로 풀스로틀 직진 (카메라가 위를 보니 시작자세가 곧 목표방향)
- 빗나가면 자동으로 위치보정 모드

## 1. 패치 덮어쓰기 + 재빌드
```bash
cp -r /home/inair/Documents/ARMS2/SW/* /home/inair/ARMS/SW/

cd /home/inair/ARMS/SW/arms_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arms_control arms_bringup arms_ui
source install/setup.bash
```

## 2. 실행
```bash
cd /home/inair/ARMS/SW
./run_arms.sh
```

## 3. 조작
- 패널 상태가 LOCK 되면 → **🚀 LAUNCH 버튼** 눌러 발사
- 드론이 풍선 반대로 가면 → Roll/Pitch sign 버튼
- 출렁/진동 → control_params.yaml 에서 kd↑(3→5), error_lpf_alpha↓(0.3→0.2)
- 못 따라감 → kp↑(5→7)

## ⚠️ 중요: 10초 안에 LAUNCH
PX4는 arm 후 ~10초 내 이륙 안 하면 자동 disarm 함 (안전장치).
LOCK 뜨면 빨리 LAUNCH 누르거나, 오래 대기하려면 PX4 콘솔(pxh>)에서:
```
param set COM_DISARM_PRFLT 0
```
(이륙 전 자동 disarm 끔)

## 발사 각도/세기 튜닝 (control_params.yaml)
- boost_throttle: 0.90   발사 추력 (더 세게 0.95~1.0)
- boost_kp: 8.0          발사각 = 픽셀오차 × 이 값 (더 많이 기울이려면 ↑)
- boost_angle_limit: 15  발사각 최대 [deg]
- boost_duration_sec: 2.0  풀스로틀 직진 시간
- boost_deviation_thresh: 0.25  이만큼 빗나가면 조기 보정전환
