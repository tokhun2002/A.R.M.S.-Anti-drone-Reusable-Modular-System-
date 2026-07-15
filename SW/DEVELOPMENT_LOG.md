# A.R.M.S. SITL 개발 로그 (Anti-drone Reusable Modular System)

> 비전 기반 자율 안티드론 요격 시스템 — PX4 SITL 시뮬레이션 개발 기록
> 환경: PX4 1.18.0 + Gazebo Harmonic + ROS2 Humble + MAVSDK C++ 2.12.2

---

## 1. 시스템 개요

드론이 상방 카메라로 풍선(적 드론 대용)을 탐지·추적하여 요격하는 SITL 시뮬레이션.

**미션 흐름:** `IDLE → SEARCH → LOCK → TRACK → FIRE → RTL`

| 상태   | 설명                               |
| ------ | ---------------------------------- |
| IDLE   | 대기 (프로펠러 OFF)                |
| SEARCH | 풍선 탐색                          |
| LOCK   | 풍선 포착, 발사 대기               |
| TRACK  | 추적 비행 (P 게인 램프 + PID 정렬) |
| FIRE   | 거리 5m 이내 도달 → 페이로드 발사  |
| RTL    | 발사 후 복귀 (명중 판정)           |

**주요 노드/토픽**

- 노드: `/arms_control_node`, `/balloon_referee`, `/fusion_detector`, `/arms_video_node`, `/arms_ui_node`, `/gz_scan_bridge`
- 토픽: `/arms/mission_state`, `/arms/launch_cmd`, `/arms/scan_raw`, `/arms/detections`

---

## 2. 핵심 제어 메커니즘

### 2-1. 시간 기반 P 램프 (kp ramp)

TRACK 진입 시 P 게인을 `kp_start → kp_max`로 `kp_ramp_sec` 동안 선형 증가.
→ 진입 직후 센 P로 중심을 잃는 것 방지.

### 2-2. 제어 부호 (물리적으로 확정)

- `roll_sign = +1.0`
- `pitch_sign = -1.0`

### 2-3. err 기반 P 자동조절 (err_sched) — 신규 추가

현재 픽셀 오차 크기 `err_mag = hypot(err_x, err_y)`에 따라 P를 자동 조절.

- 오차 작음(중앙) → P 풀파워 (빠른 마무리)
- 오차 큼(구석) → P 자동 감쇠 (발산 방지)
- `err_x`, `err_y` 양축 모두 고려 (hypot)

```
err_mag ≤ full_err  →  비율 1.0 (풀 P)
err_mag ≥ big_err   →  비율 min_ratio (최소 P)
그 사이             →  선형 보간
```

### 2-4. 거리 기반 FIRE

상방 거리센서(`/arms/scan_raw`)로 풍선까지 거리 측정, 5m 이내 도달 시 FIRE.

---

## 3. 성공 레시피 (RTL 거의 항상 성공)

> err이 0.05 이내에서 LAUNCH 시 거의 100% RTL + 명중

| 파라미터                                                   | 값    |
| ---------------------------------------------------------- | ----- |
| `control.kp_start`                                         | 33.0  |
| `control.kp_max`                                           | 130.0 |
| `control.kp_ramp_sec`                                      | 5.0   |
| `control.roll_pid.kd` / `pitch_pid.kd`                     | 0.6   |
| `control.roll_pid.ki` / `pitch_pid.ki`                     | 0.8   |
| `control.track_throttle`                                   | 0.82  |
| `control.roll_pid.output_limit` / `pitch_pid.output_limit` | 95.0  |
| `control.err_sched_enable`                                 | true  |
| `control.err_sched_full_err`                               | 0.06  |
| `control.err_sched_big_err`                                | 0.35  |
| `control.err_sched_min_ratio`                              | 0.30  |
| 풍선 고도 (`balloon_referee alt`)                          | 44.0  |
| 풍선 정지 (`balloon_referee enabled`)                      | false |

### 런타임 적용 명령

```bash
ros2 param set /balloon_referee enabled false
ros2 param set /arms_control_node control.kp_start 33.0
ros2 param set /arms_control_node control.kp_max 130.0
ros2 param set /arms_control_node control.roll_pid.kd 0.6
ros2 param set /arms_control_node control.pitch_pid.kd 0.6
ros2 param set /arms_control_node control.roll_pid.ki 0.8
ros2 param set /arms_control_node control.pitch_pid.ki 0.8
ros2 param set /arms_control_node control.track_throttle 0.82
ros2 param set /arms_control_node control.err_sched_enable true
ros2 param set /arms_control_node control.err_sched_full_err 0.06
ros2 param set /arms_control_node control.err_sched_big_err 0.35
ros2 param set /arms_control_node control.err_sched_min_ratio 0.30
ros2 param set /balloon_referee alt 44.0
```

---

## 4. 개발 타임라인

### Phase 1 — 환경 구축

- VMware Ubuntu VM + 네이티브 듀얼부팅 환경 구성
- colcon으로 6개 ARMS 패키지 빌드
- MAVSDK ComponentType 네임스페이스 에러 해결
- MAVLink 포트 설정 (udp://:14540)
- PX4 SITL + Gazebo Harmonic (LIBGL_ALWAYS_SOFTWARE=1)
- 카메라 브리지 (ros_gz_harmonic_ws 소싱)
- YOLO 풍선 모델(best.pt, 37MB) Git LFS로 관리

### Phase 2 — 제어 발산 디버깅

초기 발산(터짐) 문제의 원인 규명 및 해결:

- **제어 부호 확정:** roll_sign=+1, pitch_sign=-1
- **패널 슬라이더 오버라이드 문제:** 슬라이더가 yaml 값을 위험한 기본값(kp_max 141~150)으로 덮어씀 → range clamp 처리
- **output_limit으로 인한 flip** 방지
- **적분 windup** 해결
- 패치: 미분 LPF 필터링, 중앙 데드존 박스, 거리 기반 게인 스케줄링

### Phase 3 — 성공 레시피 도출

- 핵심 발견: **성공의 비결은 P가 높아서가 아니라 err이 0 근처라 P×0=0으로 발산하지 않는 것**
- kd 0.6이 오버슈트를 제동 → 발산 전에 풍선 도달
- 거리센서 정상 동작 확인 (d: -1.0 → 7.8 → 5.4m → FIRE)
- RTL + 거리 기반 FIRE + 명중 다회 달성

### Phase 4 — "들쭉날쭉" 원인 규명

같은 값인데 성공/실패가 갈리는 문제 추적:

- **원인 1:** run_arms 재시작 시 파라미터가 yaml 기본값으로 리셋 (kp_max 130 → 25)
- **원인 2:** 풍선 정지가 제대로 안 되어 추적 중 풍선이 도망 (err_y 급상승)
- **원인 3:** 거리가 안 줄어듦 (드론이 풍선까지 수직 접근 못 함 — 비스듬한 위치)

### Phase 5 — 빌드 위치 함정 해결

- **치명적 함정:** `cd ~/ARMS/SW`에서 colcon build 시 install이 엉뚱한 위치(SW/install)에 생성됨
- run_arms가 `arms_ws/install`을 찾는데 비어있어 패널/영상 안 뜸
- **해결:** 반드시 `cd ~/ARMS/SW/arms_ws`에서 빌드
- 0.2초 빌드 = 컴파일 스킵(stale), 정상 빌드는 수십 초

### Phase 6 — err 기반 P 자동조절 구현

- err 크기에 따라 P를 자동 조절하는 게인 스케줄링 추가
- 양축(hypot) 기반으로 x=0/y=0.3 같은 케이스도 커버
- 로그에 `(errx0.xx)` 표시로 실시간 조절 확인
- 검증: err 0.3~0.4에서도 발산 안 함 (P 자동 감쇠로 명령 ±14도 유지)

---

## 5. 코드 변경 사항 요약

### arms_control_node.cpp

**추가된 파라미터 (declare_parameter):**

- `control.err_sched_enable` (bool, default true)
- `control.err_sched_full_err` (double, default 0.06)
- `control.err_sched_big_err` (double, default 0.35)
- `control.err_sched_min_ratio` (double, default 0.30)

**추가된 멤버 변수:**

- `err_sched_enable_`, `err_sched_full_err_`, `err_sched_big_err_`, `err_sched_min_ratio_`, `err_ratio_filt_`

**핵심 로직 (kp_eff 계산부, TRACK/FIRE 블록 내):**

```cpp
if (err_sched_enable_) {
  double err_mag = std::hypot(filt_err_x_, filt_err_y_);
  double er = 1.0;
  if (err_sched_big_err_ > err_sched_full_err_ + 1e-6) {
    double u = std::clamp(
        (err_mag - err_sched_full_err_) /
        (err_sched_big_err_ - err_sched_full_err_), 0.0, 1.0);
    er = 1.0 - u * (1.0 - err_sched_min_ratio_);
  }
  err_ratio_filt_ = 0.2 * er + 0.8 * err_ratio_filt_;  // 평활화
  kp_eff *= err_ratio_filt_;
}
```

**디버그 로그 변경:** `kp=NN(errxX.XX)` 형식으로 err 조절 비율 표시

### control_params.yaml ✅ 성공 레시피 영구 저장 완료

```yaml
ki: 0.8 # (roll/pitch 양쪽)
kd: 0.6 # (roll/pitch 양쪽)
output_limit: 95.0 # 최대 자세각[deg] (roll/pitch 양쪽)
track_throttle: 0.82
kp_start: 33.0
kp_max: 130.0
kp_ramp_sec: 5.0
```

> err_sched 파라미터는 yaml에 미반영 — cpp declare_parameter 기본값으로 동작
> (enable=true, full_err=0.06, big_err=0.35, min_ratio=0.30)

### pid_controller.hpp / pid_controller.cpp

PID 관련 수정 (set_gains 등)

### arms_panel.py

패널 UI 수정 (슬라이더 범위 클램프, ki 슬라이더, 상태 표시 등)

### 커밋 안 된 변경 파일 (git status 기준)

- `arms_ws/src/arms_control/config/control_params.yaml`
- `arms_ws/src/arms_control/include/arms_control/pid_controller.hpp`
- `arms_ws/src/arms_control/src/arms_control_node.cpp`
- `arms_ws/src/arms_control/src/pid_controller.cpp`
- `tools/arms_panel.py`

### 정리 필요 (untracked, 커밋 제외 권장)

- `arms_control.zip`, `arms_patch_src.zip` (패치 zip — .gitignore 권장)
- `control_params.yaml.bak` (백업 — 제외)
- `APPLY_PATCH.md` (포함 여부 선택)

---

## 6. 빌드 방법 (정확한 절차)

```bash
cd ~/ARMS/SW/arms_ws          # ★ 반드시 arms_ws 안에서!
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding arms_control arms_msgs
source install/setup.bash
```

**주의사항:**

- 빌드는 반드시 `arms_ws` 디렉토리 안에서 (SW에서 하면 install 위치 오류)
- 빌드 후 run_arms 완전 재시작 (메모리의 옛 노드가 계속 돌 수 있음)
- 0.2초 빌드 = 스킵(stale), 정상은 수십 초
- 클린 빌드: `rm -rf build install log` 후 재빌드

---

## 7. 알려진 한계 & 다음 단계 (옵션 B)

### 현재 구조의 근본 한계

- **픽셀 기반 추적의 한계:** err은 "각도"지 "거리"가 아님 → err=0이어도 거리 34m인 상황 발생
- **비스듬한 풍선:** 드론이 수평이동을 못 해 거리를 못 좁힘
- **결론:** 풍선이 우연히 드론 바로 위에 있을 때만 안정적 성공 (들쭉날쭉의 근본 원인)

### 다음 단계 — 옵션 B (좌표 기반 요격)

픽셀(err) 대신 3D 좌표로 요격하는 방식으로 전환 예정:

```
요격 벡터 = 풍선_좌표(Gazebo) - 드론_좌표(PX4 로컬포지션)
→ 이 방향으로 속도 명령(velocity setpoint)
→ 거리 = |요격 벡터| < 2m → FIRE → RTL
```

- err 값에 **완전히 무관** → 풍선이 화면 어디 있든 100% 요격
- 영상/RTL/상태머신/명중 시퀀스는 **그대로 유지**
- SITL 데모/검증용 (실물 드론엔 적용 불가)

---

_문서 생성: 개발 진행 중 체크포인트_
