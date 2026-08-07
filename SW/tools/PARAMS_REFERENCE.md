# A.R.M.S. 요격 파라미터 레퍼런스

상황별로 실제 바뀌는 건 아래 **가변 knob 몇 개뿐**. 나머지는 전부 공통(고정)값이다.
(제어값 = `arms_control/config/control_params.yaml`, 포획반경 = 심판 `hit_radius`/패널 슬라이더)

---

## 상황별 가변 knob

| knob | 정지공 (딱 맞춤) | 2.2 m/s | **3.6 m/s (RTL 성공 검증)** |
|---|---|---|---|
| `guidance_mode` | **0 (Pursuit)** | **1 (PN)** | **1 (PN)** |
| `pn_nav_gain` (앞질러 겨냥) | — (0) | 45 | **50** |
| `pn_center_gain` (중심유지) | **45~60** (정면조준 강하게) | 25 | **15** |
| `track_throttle` (상승추력) | 0.84 | 0.84 | **0.87** |
| `lead_gain` (예측조준) | 0 | 0 | **0.0** (PN이 리드 담당) |
| `max_tilt_deg` (최대기울기) | 40 | 40~45 | **55** |
| **포획반경** `hit_radius` (딱 맞음 판정) | **1.5~2.0** | 3.0~3.5 | **3.5** |
| `fire_distance_m` (RTL 트리거) | 1~2 | 3.0 | **3.0** |

> **3.6 m/s 열 = 현재 yaml 고정값** (패널 RTL 성공 스샷 기준). error 0.00, lock 7.0s로 안정 명중.

**핵심 차이**
- **정지공** = Pursuit(추미) + 중심유지 강하게(45~60) → 흔들림 없이 정면에 딱. 포획반경 작게(1.5~2)로 진짜 접촉만 인정.
- **움직이는 공** = PN(비례항법)으로 미래 충돌점을 앞질러 겨냥(리드는 PN이 담당, lead 0). 속도 빠를수록 포획반경·상승추력·최대기울기를 조금씩 ↑.

---

## 공통 고정값 (모든 상황 동일 — 검증됨, 건들지 말 것)

### PID / 필터
- `roll_pid` / `pitch_pid`: kp **130**, ki 0, kd **1.0**, output_limit 45
- `error_lpf_alpha` 0.25, `deriv_lpf_alpha` 0.25, `deadzone` 0.04
- `control_rate_hz` 50

### 추력
- `throttle` 0.55, `hover_throttle` 0.62, `pursuit_gate` 0.25

### ACRO(각속도) 자세제어
- `acro_mode` true, `max_rate_dps` 220
- `att_p` **3.5** (올리면 자가발진 — 고정)
- `att_roll_sign` +1, `att_pitch_sign` −1
- `att_comp` 0.012 (자세보정 LOS)
- `roll_sign` +1, `pitch_sign` +1

### PN 상태추정 (움직이는 공)
- `pn_alpha` 0.35, `pn_beta` 0.02 (리드 매끈하게), `pn_los_clamp` 1.5
- `pn_commit_dist` 0 (종말부스트 OFF — 발진 원인이라 끔)

### 예측(lead)
- `lead_gain` 0.0 (PN이 리드를 담당하므로 별도 예측조준 OFF — 3.6m/s 검증값), `lead_dist` 0

### 미션 / 자동발사
- `sitl_auto_launch` true, `auto_launch_delay_sec` 0.5, `lock_duration_sec` 0.3

---

## 명중 시 동작 (심판)
- 실제 충돌(중심거리 < 포획반경) → `/arms/hit` 발행 → 제어노드 **FIRE→RTL**
- 풍선이 **맞은 방향+위로 22 m/s로 1.5초** 튕겨 날아간 뒤 사라짐 (실제 충돌 물리반응)
- 로그: `직격 접근거리 X.Xm (최소 Y.Ym)` — 최소값 ≤ 포획반경이면 명중

---

## 튜닝 순서 메모
1. 정지공부터 안정화(안 떨고 중앙에) → kp, 그다음 kd → att_p → 중심유지(center_gain).
2. 움직이는 공: `guidance_mode` 1(PN)로 전환, nav 45 / center 25에서 시작.
3. 못 따라가면(뒤처짐) → `track_throttle`↑, `max_tilt`↑.
4. 안 맞으면(최소접근 큼) → 포획반경 살짝↑ 또는 위 3번.
5. 흔들리면 → `pn_beta`↓ 또는 `pn_center_gain`↓.
