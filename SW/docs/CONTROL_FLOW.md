# A.R.M.S. 제어 흐름

_arms_control_node 중심으로 검출 → 상태 판단 → 비행 명령까지의 전체 경로 설명_

---

## 목차

1. [전체 파이프라인 개요](#1-전체-파이프라인-개요)
2. [입력: 검출 결과](#2-입력-검출-결과)
3. [상태 머신](#3-상태-머신)
4. [PID 제어](#4-pid-제어)
5. [스로틀 제어](#5-스로틀-제어)
6. [CRSF 출력 채널 매핑](#6-crsf-출력-채널-매핑)
7. [SITL 브리지 (MAVLink 변환)](#7-sitl-브리지-mavlink-변환)
8. [수동 조종 모드](#8-수동-조종-모드)
9. [게인 스케줄링](#9-게인-스케줄링)
10. [파라미터 일람](#10-파라미터-일람)

---

## 1. 전체 파이프라인 개요

```
카메라 영상
    │
    ▼
arms_detection_node  ─── /arms/detections ──▶  arms_control_node (30 Hz)
                                                       │
조종기 입력                                            │  상태 머신
/arms/command (Joy) ──────────────────────────▶       │  PID (roll, pitch)
                                                       │  스로틀 제어
라이다 / 거리 센서                                     │
/arms/distance ────────────────────────────────▶       │
                                                       │
                                               CRSF 시리얼 출력
                                                 /tmp/crsf_tx (SITL)
                                                 /dev/ttyUSB0 (실기체)
                                                       │
                              ┌────────────────────────┘
                              │
               [SITL]         │          [실기체]
          sitl_bridge_node    │     ELRS TX 모듈
          MAVLink UDP ──▶ PX4 │     ELRS RX → FC (Stabilized 모드)
```

- **제어 주기**: 30 Hz (`control_rate_hz` 파라미터)
- **모드**: Auto(PID) / Manual(조종기 축 값 직접 전달) — `/arms/command` buttons[2] 토글

---

## 2. 입력: 검출 결과

`arms_detection_node`가 `/arms/detections` (arms_msgs/DetectionArray)를 발행한다.

각 BoundingBox는 `x_center`, `y_center`가 **[0, 1] 정규화 픽셀 좌표**이고,
제어에서 쓰는 오차는 이를 프레임 중앙(0.5) 기준으로 변환한다.

```
error_x = x_center - 0.5   (양수 = 표적이 오른쪽)
error_y = y_center - 0.5   (양수 = 표적이 아래쪽)
```

검출 결과가 없거나 confidence < 임계값이면 `on_target_lost()` 호출 → 연속 누락 횟수 카운트.

---

## 3. 상태 머신

`state_machine.cpp` 구현. 매 제어 루프에서 상태를 읽어 PID/CRSF를 결정한다.

```
                   시작 (auto_armed_=false)
                          │
                    [Auto 모드 진입]
                          ▼
          IDLE ─────────────────▶ SEARCH
                 arm() 즉시 1회
                          │
                  표적 N 프레임 연속 검출
                  (lock_duration_sec 이상)
                          ▼
                        LOCK   ◀── 표적 상실 시 SEARCH 복귀
                          │
                  [LAUNCH 버튼]
                          ▼
                        TRACK  ◀── 표적 상실 시 SEARCH 복귀
                          │
                  거리 < fire_distance_m
                          ▼
                         FIRE ─────▶ RTL ─────▶ IDLE
                      (CH8 HIGH 1s)    (CH6 HIGH)  (착륙 후)
```

| 상태 | FC arm | PID 활성 | 스로틀 |
|------|--------|----------|--------|
| IDLE | OFF | — | 0 |
| SEARCH | OFF | — | 0 |
| LOCK | ON | — | 0 |
| TRACK | ON | **ON** | track_throttle |
| FIRE | ON | ON | track_throttle (정렬 유지) |
| RTL | ON | — | 0 (FC AUTO LAND) |

> **IDLE → SEARCH**: Auto 모드이면 시작 시 즉시 1회 전이 (FC는 아직 disarm).  
> **LOCK → TRACK**: `/arms/command` buttons[3] 상승 에지(LAUNCH 버튼 또는 `sitl_auto_launch` 파라미터).  
> **kill 스위치(CH7)**: 상태 머신과 무관 — FC가 하드웨어 레벨에서 모터 직접 차단.

### 표적 상실 처리

`lost_frames_threshold` 프레임 연속 누락 → LOCK/TRACK → SEARCH 복귀, PID 리셋.

---

## 4. PID 제어

TRACK/FIRE 상태에서만 동작. roll과 pitch 각각 독립 PID.

### 4.1 오차 전처리

```
raw_ex, raw_ey  (검출 오차)
    │
    ▼  LPF (error_lpf_alpha, 기본 0.3)
filt_err_x, filt_err_y
    │
    ▼  데드존 제거 (deadzone, 기본 0.04)
ex, ey  → PID 입력
```

LPF: `filt = α·raw + (1-α)·filt`  (α 작을수록 부드러움, 응답 느림)

### 4.2 PID 계산

```
output = Kp·e  +  Ki·∫e·dt  +  Kd·(de/dt)

미분항: d_filt = α_d·raw_d + (1-α_d)·d_filt  (deriv_lpf_alpha, 기본 0.25)
적분항: 안티-와인드업 — output_limit 이상 쌓이지 않게 클램핑
```

### 4.3 각도 → CRSF 변환

PID 출력은 **기울기 각도 [deg]**. CRSF 값으로 변환:

```
norm = roll_deg / crsf_max_angle_  (기본 35°)
crsf = CRSF_CENTER + norm × (CRSF_MAX - CRSF_CENTER)   (양수 방향)
     = CRSF_CENTER + norm × (CRSF_CENTER - CRSF_MIN)   (음수 방향)
```

`roll_sign`, `pitch_sign` (±1.0): 카메라/기체 방향에 따라 제어 방향 뒤집기.  
pitch는 roll 대비 게인을 1.6배 적용 (pitch 축 응답 보정).

---

## 5. 스로틀 제어

TRACK/FIRE 상태의 스로틀은 고정값이 아니라 **정렬 상태 + 거리**에 따라 결정된다.

### 5.1 정렬 게이트 (align_locked)

```
오차 크기(emag) < align_thr(0.10) 이 처음 달성되면 align_locked = true
```

- `align_locked = false`: hover 추력(0.62) 유지 — 표적 방향으로 정렬 중
- `align_locked = true`: 아래 거리 제어로 진입

### 5.2 거리 기반 스로틀

```
align_locked 이후에도 xy_gate(0.22) 이상이면 → hover 복귀 (x/y 오차 커졌을 때)

xy_gate 이내이고 거리 데이터 유효할 때:
  d > fire_d + 3 m     → track_throttle  (상승 접근)
  fire_d < d ≤ fire_d+3 → 선형 보간      (감속 접근)
  d ≤ fire_d           → hover           (사거리 도달, FIRE 전환 대기)

거리 데이터 없을 때: track_throttle 유지
```

거리 입력: `/arms/distance` (sensor_msgs/Range) 또는 `/arms/scan_raw` (LaserScan) 중 최솟값.  
0.5초 이내 수신된 값만 유효(`distance_valid()`).

---

## 6. CRSF 출력 채널 매핑

30 Hz로 `CrsfOutput::send()` 호출. 26바이트 CRSF RC 프레임을 시리얼로 전송.

| CH | 내용 | Auto 모드 | Manual 모드 |
|----|------|-----------|-------------|
| 1 | Roll | PID roll 각도 → norm | joy_axes_[0] |
| 2 | Pitch | PID pitch 각도 → norm | joy_axes_[1] |
| 3 | Throttle | 스로틀 제어 결과 | joy_axes_[2] |
| 4 | Yaw | CRSF_CENTER (고정) | joy_axes_[3] |
| 5 | Arm | LOCK 이상 → MAX, 아니면 MIN | 동일 |
| 6 | Land | RTL 상태 or 수동 LAND → MAX | 동일 |
| 7 | Kill | buttons[0] HIGH → MAX | 동일 |
| 8 | Launch/Fire | FIRE 진입 시 1s HIGH | buttons[3] |

CRSF 범위: MIN=172, CENTER=992, MAX=1811 (11-bit, 16채널 packed)

---

## 7. SITL 브리지 (MAVLink 변환)

실기체에서는 CRSF 시리얼이 ELRS TX 모듈로 직접 연결되지만,  
SITL에서는 `sitl_bridge_node.py`가 `/tmp/crsf_rx` 포트를 읽어 MAVLink로 변환한다.

```
arms_control_node
  /tmp/crsf_tx ──(socat PTY 쌍)──▶ /tmp/crsf_rx
                                          │
                                  sitl_bridge_node
                                          │
                              MAVLink UDP udpin:0.0.0.0:14540
                                          │
                                        PX4 SITL
```

### 브리지 동작

1. **연결**: PX4 HEARTBEAT 대기 후 Stabilized 모드 설정
2. **CRSF 파싱**: 26바이트 프레임 → 16채널 언팩 → μs 변환 (`172~1811 → 1000~2000 μs`)
3. **RC_CHANNELS_OVERRIDE**: 50 Hz로 8채널 MAVLink 메시지 송신 → PX4가 이를 SOURCE_RC로 처리
4. **Arm/Disarm**: CH5 레벨 모니터링 → 원하는 상태와 실제 상태(`HEARTBEAT.base_mode & 0x80`) 불일치 시 1초 간격으로 `MAV_CMD_COMPONENT_ARM_DISARM` 재시도
5. **착륙**: CH6 상승 에지 → `MAV_CMD_DO_SET_MODE` (AUTO LAND)

### PX4 파라미터 (SITL 전용)

```
COM_RC_IN_MODE = 3   # RC or MAVLink — SOURCE_RC override 허용
RC_CHAN_CNT    = 8   # 채널 수 > 0 이어야 manual_control_setpoint.valid = true
RC1~8 MIN/TRIM/MAX = 1000/1500/2000   # 캘리브레이션 — 미설정 시 모든 채널 최댓값으로 읽힘
RC_MAP_ARM_SW  = 0   # 브리지가 MAVLink로 직접 처리 (FC 자체 RC 스위치 감시 비활성)
NAV_DLL_ACT    = 0   # GCS 없는 standalone 비행 — datalink failsafe 비활성화
NAV_RCL_ACT    = 0   # RC 소실 failsafe 비활성화
```

---

## 8. 수동 조종 모드

`/arms/command` buttons[2] 상승 에지마다 `joy_manual_mode_` 토글.

- **Auto**: PID 출력 + 상태 기반 스로틀을 CH1~3에 출력
- **Manual**: `/arms/command` axes[0~3]을 CH1~4에 직접 출력, 상태 머신 동작은 유지

SITL에서는 `arms_command_node` tkinter GUI의 조종기 창(`ControllerGUI`)에서 스틱 드래그로 axes를 발행.  
실기체에서는 ESP32 모듈이 ADS1115(짐벌 4축) + GPIO(스위치)를 읽어 USB Serial로 Jetson에 전달 → `controller_input_node`가 `/arms/command`를 발행.

---

## 9. 게인 스케줄링

TRACK 진입 후 Kp는 고정이 아니라 3가지 요인에 의해 동적으로 결정된다.

### 9.1 시간 기반 P 램프

TRACK 진입 시 Kp를 `kp_start`부터 시작해 `kp_ramp_sec` 동안 `kp_max`까지 선형으로 증가.

```
elapsed = 현재 - TRACK 진입 시각
ramp_t  = clamp(elapsed / kp_ramp_sec, 0, 1)
kp_now  = kp_start + ramp_t × (kp_max - kp_start)
```

진입 직후 급격한 틱 방지용.

### 9.2 거리 기반 게인 스케줄링

표적이 가까울수록 Kp를 줄여 과도 응답 방지.

```
d = 라이다 거리
ratio = gain_sched_min_ratio + (d / gain_sched_near_m) × (1 - gain_sched_min_ratio)
      (0~1 클램프)
gain_ratio_filt = 0.15·ratio + 0.85·gain_ratio_filt  (LPF)
kp_eff = kp_now × gain_ratio_filt
```

거리 데이터 없으면 ratio=1 (full gain).

### 9.3 오차 크기 기반 P 억제

오차가 클 때 Kp를 줄여 오버슈트 방지 (`err_sched_enable = true`일 때).

```
err_mag = hypot(filt_err_x, filt_err_y)
u       = clamp((err_mag - err_sched_full_err) / (err_sched_big_err - err_sched_full_err), 0, 1)
er      = 1 - u × (1 - err_sched_min_ratio)
err_ratio_filt = 0.2·er + 0.8·err_ratio_filt
kp_eff  = kp_eff × err_ratio_filt
```

오차가 `err_sched_full_err` 이하이면 억제 없음(er=1), `err_sched_big_err` 이상이면 최소 비율(`err_sched_min_ratio`)로 억제.

최종 적용:
```
pid_roll.Kp  = kp_eff
pid_pitch.Kp = kp_eff × 1.6   (pitch 보정 배수)
```

---

## 10. 파라미터 일람

`arms_control_node` 런타임 변경 가능 (`ros2 param set /arms_control_node ...`).

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `control.roll_pid.kp` | 15.0 | Roll P (실제 Kp는 게인 스케줄링으로 변동) |
| `control.roll_pid.ki` | 0.5 | Roll I |
| `control.roll_pid.kd` | 1.0 | Roll D |
| `control.roll_pid.output_limit` | 90.0 | Roll 최대 각도 [deg] |
| `control.pitch_pid.*` | (동일) | Pitch PID (Kp는 1.6배 추가 적용) |
| `control.throttle` | 0.55 | 기본 hover 추력 (미사용, track_throttle로 대체) |
| `control.track_throttle` | 0.60 | TRACK 상승 추력 (0~1) |
| `control.roll_sign` | 1.0 | Roll 제어 방향 (±1) |
| `control.pitch_sign` | 1.0 | Pitch 제어 방향 (±1) |
| `control.error_lpf_alpha` | 0.3 | 오차 LPF 계수 (1=즉시, 작을수록 부드러움) |
| `control.kp_start` | 60.0 | 시간 램프 시작 Kp |
| `control.kp_max` | 150.0 | 시간 램프 최대 Kp |
| `control.kp_ramp_sec` | 5.0 | Kp 증가 시간 [s] |
| `control.deadzone` | 0.04 | 중앙 데드존 (정규화 오차 단위) |
| `control.deriv_lpf_alpha` | 0.25 | 미분항 LPF 계수 |
| `control.gain_sched_near_m` | 4.0 | 거리 게인 억제 기준 거리 [m] |
| `control.gain_sched_min_ratio` | 0.35 | 거리 억제 최소 게인 비율 |
| `control.err_sched_enable` | true | 오차 기반 게인 억제 활성 |
| `control.err_sched_full_err` | 0.06 | 억제 시작 오차 크기 |
| `control.err_sched_big_err` | 0.35 | 최대 억제 오차 크기 |
| `control.err_sched_min_ratio` | 0.55 | 오차 억제 최소 게인 비율 |
| `control.control_rate_hz` | 30.0 | 제어 루프 주기 |
| `crsf.max_angle_deg` | 35.0 | PID 출력 → CRSF 변환 기준 각도 |
| `mission.detection_confidence_threshold` | 0.65 | 검출 신뢰도 임계값 |
| `mission.lock_duration_sec` | 2.0 | LOCK 전환까지 연속 검출 필요 시간 |
| `mission.lost_frames_threshold` | 10 | 표적 상실 판정 누락 프레임 수 |
| `mission.fire_distance_m` | 5.0 | FIRE 전환 사거리 [m] |
