# 드론 자동주행 동작 모드 알고리즘

## 1. 한눈에 보는 요약

### 1.1 전체 우선순위

1. **KILL**: 모든 동작 즉시 정지
2. **MANUAL / AUTO**: 수동·자동 운용 선택
3. **AUTO 내부 상태**: `IDLE`, `SEARCH`, `LOCK`, `TRACK`

### 1.2 스위치 역할

| 신호 | 종류 | 처리 방식 | 역할 |
|---|---|---|---|
| `kill` | 토글 | 현재 값을 매 주기 검사 | 전체 시스템 강제 정지 |
| `mode` | 토글 | 상승·하강엣지 검사 | MANUAL / AUTO 전환 |
| `eland` | 토글 | 상승·하강엣지 검사 | 자동 동작 시작 및 종료 |
| `fire` | 순간 스위치 | 상승엣지 검사 | `LOCK`에서 `TRACK`으로 전환 |

이 문서에서는 다음과 같이 가정한다.

```text
mode = 1 : MANUAL
mode = 0 : AUTO
```

### 1.3 자동모드 상태

| 상태 | 카메라 | 제어기 출력 적용 비율 | 전환 조건 |
|---|---|---:|---|
| `IDLE` | 목표물 탐색 OFF | 0% | `eland` 상승엣지 → `SEARCH` |
| `SEARCH` | 목표물 탐색 ON | 5% | 5프레임 연속 감지 → `LOCK`<br>`eland` 하강엣지 → `IDLE` |
| `LOCK` | 목표물 고정·추적 | 5% | `fire` 상승엣지 + 목표 유효 → `TRACK`<br>10프레임 연속 미감지 → `SEARCH`<br>`eland` 하강엣지 → `IDLE` |
| `TRACK` | 목표물 추적 | 100% | 목표물 상실 → `MANUAL`<br>`eland` 하강엣지 → `IDLE` |

출력 비율은 모터 최대출력의 비율이 아니라, **제어기가 계산한 명령에 곱하는 비율**이다.

```cpp
IDLE   : motor_cmd = 0.0;
SEARCH : motor_cmd = controller_cmd * 0.05;
LOCK   : motor_cmd = controller_cmd * 0.05;
TRACK  : motor_cmd = controller_cmd * 1.00;
```

### 1.4 핵심 안전 규칙

- `kill == 1`이면 다른 조건과 관계없이 즉시 정지하고 자동 상태를 `IDLE`로 초기화한다.
- KILL 해제 후에는 반드시 `IDLE`에서 시작한다.
- 수동에서 자동으로 전환한 뒤에도 반드시 `IDLE`에서 시작한다.
- 위 두 경우 모두 `eland`가 이미 1이면 바로 `SEARCH`로 들어가지 않는다. `eland`를 0으로 내린 뒤 다시 0→1로 올려야 한다.
- `TRACK`에서 목표물을 잃으면 강제로 `MANUAL`로 전환한다.
- 강제 수동 전환 후에는 `mode`를 `0→1→0`으로 다시 조작해야 AUTO에 재진입할 수 있다.
- AUTO에 재진입해도 `IDLE`부터 시작하며, 새로운 `eland` 상승엣지가 필요하다.

---

## 2. 상세 동작 규칙

### 2.1 스위치 엣지 판정

```cpp
bool mode_rising   = (mode == 1 && prev_mode == 0);
bool mode_falling  = (mode == 0 && prev_mode == 1);

bool eland_rising  = (eland == 1 && prev_eland == 0);
bool eland_falling = (eland == 0 && prev_eland == 1);

bool fire_rising   = (fire == 1 && prev_fire == 0);
```

`fire`가 다음처럼 일정 시간 1을 유지해도 상승엣지는 한 번만 발생한다.

```text
0 0 0 1 1 1 0 0 0
      ↑
    1회
```

`kill`은 엣지가 아니라 현재 값을 매 제어 주기마다 확인한다.

ESP32 조종기에서 입력 튐과 순간적인 잘못된 값은 미리 처리된 것으로 가정한다.

### 2.2 전체 제어 구조

```cpp
if (kill == 1) {
    auto_state = IDLE;
    stop_all_outputs();
}
else if (control_mode == MANUAL) {
    auto_state = IDLE;
    run_manual_control();
}
else {
    run_auto_state_machine();
}
```

#### KILL이 켜진 경우

- 모터 출력 정지
- 카메라 탐색 및 자동주행 정지
- 자동 상태를 `IDLE`로 초기화
- KILL 해제 후 새로운 `eland` 상승엣지가 있어야 `SEARCH` 진입

#### MODE가 전환된 경우

- `mode` 상승엣지: AUTO → MANUAL
- `mode` 하강엣지: MANUAL → AUTO
- 수동·자동이 전환될 때 자동 상태를 `IDLE`로 초기화
- MANUAL → AUTO 전환 후 새로운 `eland` 상승엣지가 있어야 자동 동작 시작

---

## 3. 자동모드 상태별 동작

### 3.1 IDLE

- 카메라 목표물 탐색 OFF
- 모터 출력 0%
- `eland` 상승엣지가 발생하면 `SEARCH`로 전환

```cpp
if (eland_rising) {
    auto_state = SEARCH;
}
```

### 3.2 SEARCH

- 카메라 목표물 탐색 ON
- 제어기 출력에 `0.05`를 곱해 적용
- 목표물이 5프레임 연속 감지되면 `LOCK`으로 전환
- `eland` 하강엣지가 발생하면 `IDLE`로 전환

```cpp
if (eland_falling) {
    auto_state = IDLE;
}
else if (detected_count >= 5) {
    auto_state = LOCK;
}
```

### 3.3 LOCK

- 카메라로 목표물을 고정하고 추적
- 제어기 출력에 `0.05`를 곱해 적용
- 목표물이 10프레임 연속 미감지되면 `SEARCH`로 복귀
- 유효한 목표물이 있을 때 `fire` 상승엣지가 발생하면 `TRACK`으로 전환
- `eland` 하강엣지가 발생하면 `IDLE`로 전환

전환 우선순위는 다음 순서로 처리한다.

1. `eland` 하강엣지
2. 목표물 10프레임 연속 미감지
3. `fire` 상승엣지와 목표 유효성 확인

```cpp
if (eland_falling) {
    auto_state = IDLE;
}
else if (lost_count >= 10) {
    auto_state = SEARCH;
}
else if (fire_rising && target_valid) {
    auto_state = TRACK;
}
```

따라서 목표물을 놓친 순간 `fire`가 눌려도 `TRACK`으로 진입하지 않는다.

### 3.4 TRACK

- 카메라 목표물 추적 유지
- 제어기 출력을 축소하지 않고 그대로 적용
- `eland` 하강엣지가 발생하면 `IDLE`로 전환
- 목표물을 상실하면 강제로 `MANUAL`로 전환

```cpp
if (eland_falling) {
    auto_state = IDLE;
}
else if (track_target_lost) {
    control_mode = MANUAL;
    auto_state = IDLE;
    auto_rearm_required = true;
}
```

`track_target_lost`의 구체적인 판정 기준은 카메라 추적 모듈에서 별도로 정한다.

---

## 4. 목표물 검출 판정

초기 기준은 다음과 같다.

```text
5프레임 연속 감지    → SEARCH에서 LOCK으로 전환
10프레임 연속 미감지 → LOCK에서 SEARCH로 전환
목표물 상실 판정      → TRACK에서 강제 MANUAL 전환
```

연속 감지 여부를 확인하기 위해 반대쪽 카운터는 매번 초기화한다.

```cpp
if (target_detected) {
    detected_count++;
    lost_count = 0;
}
else {
    lost_count++;
    detected_count = 0;
}
```

상태가 바뀌면 두 카운터를 모두 초기화한다.

```cpp
detected_count = 0;
lost_count = 0;
```

---

## 5. TRACK 목표물 상실 후 AUTO 재진입

`TRACK`에서 목표물을 잃으면 소프트웨어가 강제로 `MANUAL`로 전환한다. 이때 물리 `mode` 스위치가 AUTO 위치인 0에 남아 있을 수 있으므로 즉시 AUTO로 복귀하지 않도록 재진입을 잠근다.

```cpp
auto_rearm_required = true;
```

다시 AUTO로 들어가려면 다음 순서가 필요하다.

1. `mode`를 0에서 1로 올려 상승엣지 발생
2. 재진입 잠금 해제
3. `mode`를 1에서 0으로 내려 하강엣지 발생
4. AUTO의 `IDLE` 상태로 진입
5. `eland`를 새로 0→1로 올려 `SEARCH` 시작

```cpp
if (mode_rising) {
    control_mode = MANUAL;
    auto_state = IDLE;
    auto_rearm_required = false;
}
else if (mode_falling && !auto_rearm_required) {
    control_mode = AUTO;
    auto_state = IDLE;
}
```

---

## 6. 전체 FSM 의사코드

```cpp
enum class ControlMode {
    MANUAL,
    AUTO
};

enum class AutoState {
    IDLE,
    SEARCH,
    LOCK,
    TRACK
};

ControlMode control_mode = ControlMode::MANUAL;
AutoState auto_state = AutoState::IDLE;

bool auto_rearm_required = false;

void initializeControl()
{
    // 시작할 때 물리 mode 스위치 위치를 논리 모드에 반영한다.
    control_mode = (mode == 1)
        ? ControlMode::MANUAL
        : ControlMode::AUTO;

    auto_state = AutoState::IDLE;
    auto_rearm_required = false;
    resetTargetCounters();

    // 시작 직후 가짜 엣지가 생기지 않도록 현재 값을 저장한다.
    savePreviousInputs();
}

void updateControl()
{
    bool mode_rising   = (mode == 1 && prev_mode == 0);
    bool mode_falling  = (mode == 0 && prev_mode == 1);
    bool eland_rising  = (eland == 1 && prev_eland == 0);
    bool eland_falling = (eland == 0 && prev_eland == 1);
    bool fire_rising   = (fire == 1 && prev_fire == 0);

    // mode 상태는 KILL 중에도 물리 스위치와 동기화한다.
    // 단, 실제 출력 제어에서는 KILL이 항상 가장 높은 우선순위를 가진다.
    if (mode_rising) {
        control_mode = ControlMode::MANUAL;
        auto_state = AutoState::IDLE;
        auto_rearm_required = false;
        resetTargetCounters();
    }
    else if (mode_falling && !auto_rearm_required) {
        control_mode = ControlMode::AUTO;
        auto_state = AutoState::IDLE;
        resetTargetCounters();
    }

    // 1순위: KILL
    if (kill == 1) {
        auto_state = AutoState::IDLE;
        resetTargetCounters();
        stopAllOutputs();
        savePreviousInputs();
        return;
    }

    // 2순위: MANUAL / AUTO
    if (control_mode == ControlMode::MANUAL) {
        auto_state = AutoState::IDLE;
        runManualControl();
        savePreviousInputs();
        return;
    }

    // 3순위: AUTO 내부 상태
    updateTargetCounters();

    switch (auto_state) {

    case AutoState::IDLE:
        setCameraSearch(false);
        setMotorScale(0.0);

        if (eland_rising) {
            auto_state = AutoState::SEARCH;
            resetTargetCounters();
        }
        break;

    case AutoState::SEARCH:
        setCameraSearch(true);
        setMotorScale(0.05);

        if (eland_falling) {
            auto_state = AutoState::IDLE;
            resetTargetCounters();
        }
        else if (detected_count >= 5) {
            auto_state = AutoState::LOCK;
            resetTargetCounters();
        }
        break;

    case AutoState::LOCK:
        setCameraSearch(true);
        setMotorScale(0.05);

        if (eland_falling) {
            auto_state = AutoState::IDLE;
            resetTargetCounters();
        }
        else if (lost_count >= 10) {
            auto_state = AutoState::SEARCH;
            resetTargetCounters();
        }
        else if (fire_rising && target_valid) {
            auto_state = AutoState::TRACK;
            resetTargetCounters();
        }
        break;

    case AutoState::TRACK:
        setCameraSearch(true);
        setMotorScale(1.0);

        if (eland_falling) {
            auto_state = AutoState::IDLE;
            resetTargetCounters();
        }
        else if (track_target_lost) {
            control_mode = ControlMode::MANUAL;
            auto_state = AutoState::IDLE;
            auto_rearm_required = true;
            resetTargetCounters();
        }
        break;
    }

    savePreviousInputs();
}
```

---

## 7. 구현 시 확인할 항목

- `prev_mode`, `prev_eland`, `prev_fire`는 모든 판정이 끝난 뒤 갱신한다.
- 프로그램 시작 시 이전 입력값을 현재 스위치 값으로 초기화해 가짜 엣지를 방지한다.
- 상태가 전환될 때 목표물 감지·미감지 카운터를 초기화한다.
- `kill == 1`이면 카메라 처리, 자동주행 계산, 모터 명령을 모두 무효화한다.
- KILL 중에도 `mode` 엣지는 기록하여 물리 스위치와 논리 모드가 어긋나지 않게 한다. 실제 출력은 계속 정지한다.
- KILL 해제 및 AUTO 재진입 시 `eland`의 새로운 상승엣지를 요구한다.
- 카메라 FPS에 따라 5프레임과 10프레임의 실제 시간이 달라지므로 시험 후 조정한다.
- `TRACK`의 목표물 상실 판정 기준은 카메라 추적 모듈에서 별도로 정한다.
