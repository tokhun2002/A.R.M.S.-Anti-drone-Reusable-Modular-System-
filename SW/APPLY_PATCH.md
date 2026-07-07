# A.R.M.S. 추적 안정화 패치 (B: 중앙유지 / C: 근접발산)

## 바뀐 파일
- arms_ws/src/arms_control/include/arms_control/pid_controller.hpp  (미분 LPF)
- arms_ws/src/arms_control/src/pid_controller.cpp                   (미분 LPF 적용)
- arms_ws/src/arms_control/src/arms_control_node.cpp                (데드존 + 거리 게인 스케줄링)
- arms_ws/src/arms_control/config/control_params.yaml              (부호 확정 + 안전값 + 새 파라미터)
- tools/arms_panel.py                                              (기본값 정리 + 새 슬라이더 3개)

## 적용 방법
1) 이 zip 을 ~/ARMS/SW 에서 풀어 덮어쓰기:
   cd ~/ARMS/SW
   unzip -o ~/Downloads/arms_patch_out.zip      # (zip 받은 경로에 맞게)

2) 재빌드 (yaml/config 도 install 로 다시 복사됨):
   cd ~/ARMS/SW
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-select arms_control
   source arms_ws/install/setup.bash

3) 실행:
   ./run_arms.sh

## 핵심 변경 요약
- 부호 확정: roll_sign +1, pitch_sign -1  (화살표 정렬되는 값)
- 게인 안전값: kp_start 8 / kp_max 25 / output_limit 30 / kd 0.6
- 미분 LPF: kd 올려도 노이즈로 안 떨림 → 오버슈트 제동 가능 (B)
- 중앙 데드존(박스): |오차|<0.04 면 명령 0 → 중앙에서 헌팅/미세떨림 멈춤 (B)
- 거리 게인 스케줄링: 4m 안으로 들어오면 P 를 최소 35%까지 자동 감쇠 (C)
  · 거리 센서 신호 없으면 자동으로 풀게인(안전). /arms/distance 또는 /arms/scan_raw 수신 필요.

## 패널 새 슬라이더
- 중앙 박스 (데드존)        : 0.04 기본. 키우면 중앙 더 둔감(안 떨림), 너무 키우면 부정확
- 근접 감쇠 시작 거리 [m]   : 4.0 기본. 이 거리 안에서 P 줄이기 시작
- 최소 게인 [%] (근접시)    : 35 기본. 가장 가까울 때 P 비율. 100=감쇠 끔

## 튜닝 순서 (정지 풍선부터)
1. 풍선 정지 → LAUNCH → 중앙으로 모이고 멈추는지 (안 떨면 OK)
2. 중앙에서 떨면 → kd ↑ (0.6→0.9) 또는 중앙 박스 ↑ (0.04→0.06)
3. 못 따라가면 → 최대 P ↑ (25→40)
4. 가까이서 떨면 → 최소 게인 [%] ↓ (35→25) 또는 근접거리 ↑ (4→6)
5. 잘 되면 풍선 비행 시작(느린 모드)로 이동 표적 테스트
