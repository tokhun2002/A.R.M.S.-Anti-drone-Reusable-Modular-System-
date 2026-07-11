# ESP32-S3 조종기 송신기

ESP32-S3에서 짐벌 2개의 아날로그 축 4개와 스위치 4개의 상태를 읽고, 조종 명령을 UART 패킷으로 변환하여 Jetson에 100 Hz로 전송하는 조종기 펌웨어입니다.

## 1. 주요 기능

- 짐벌 입력 4축
  - Throttle
  - Roll
  - Pitch
  - Yaw
- 스위치 입력 4개
  - Fire
  - Auto/Manual Mode
  - Emergency Landing
  - Kill
- 짐벌 입력에 1차 저역통과필터(LPF) 적용
- 중앙복귀 축에 데드밴드 적용
- 실제 조종기 측정 결과를 반영한 ADC 보정값 적용
- ESP32-S3 UART1을 통해 Jetson으로 ASCII 패킷 전송
- USB Serial Monitor를 이용한 디버깅 지원

---

## 2. 좌표계 및 회전 방향

<img width="886" height="666" alt="image" src="https://github.com/user-attachments/assets/ce088d08-abd4-4d44-a44c-a36500f112f2" />

드론의 중심을 기준으로 다음 축을 사용합니다.

- **X축**: 드론의 앞뒤 방향으로 뻗은 축
- **Y축**: 드론의 좌우 방향으로 뻗은 축
- **Z축**: 드론의 중심에서 수직 위쪽으로 뻗은 축
- **Roll**: X축을 중심으로 하는 회전
- **Pitch**: Y축을 중심으로 하는 회전
- **Yaw**: Z축을 중심으로 하는 회전

양수와 음수 방향은 위 그림의 축 방향 및 회전 화살표를 기준으로 설정되어 있습니다. 현재 펌웨어의 `invert` 설정도 실제 조종기 입력이 이 좌표계와 일치하도록 구성되어 있습니다.

짐벌 설치 방향이나 배선을 변경하면 축 방향이 반대로 출력될 수 있습니다. 이 경우 해당 축의 `AxisConfig`에서 `invert` 값을 변경합니다.

```cpp
false  // 현재 방향 유지
true   // 출력 부호 반전
```

---

## 3. 사용 하드웨어

- ESP32-S3 N16R8
- 아날로그 짐벌 2개
  - 중앙복귀 짐벌 1개
  - 스로틀용 짐벌 1개
- 스위치 4개
- NVIDIA Jetson
- USB 케이블
- ESP32와 Jetson UART 연결선

> ESP32-S3 ADC 입력 핀에는 5 V를 연결하면 안 됩니다. 짐벌 출력 신호는 반드시 0~3.3 V 범위여야 합니다.

---

## 4. 핀 연결

### 4.1 짐벌

| 기능 | ESP32-S3 핀 | 설명 |
|---|---:|---|
| Roll | GPIO1 | 중앙복귀 짐벌 X축 |
| Pitch | GPIO2 | 중앙복귀 짐벌 Y축 |
| Throttle | GPIO4 | 스로틀 짐벌 Y축 |
| Yaw | GPIO5 | 스로틀 짐벌 X축 |

각 짐벌의 전원은 다음과 같이 연결합니다.

```text
짐벌 VCC -> ESP32-S3 3V3
짐벌 GND -> ESP32-S3 GND
```

### 4.2 스위치

| 기능 | ESP32-S3 핀 | 연결 |
|---|---:|---|
| Kill | GPIO15 | GPIO15 ↔ 스위치 ↔ GND |
| Emergency Landing | GPIO16 | GPIO16 ↔ 스위치 ↔ GND |
| Auto/Manual Mode | GPIO17 | GPIO17 ↔ 스위치 ↔ GND |
| Fire | GPIO18 | GPIO18 ↔ 스위치 ↔ GND |

스위치 입력은 `INPUT_PULLUP`을 사용합니다.

```text
물리 스위치 OFF 또는 열림 = GPIO HIGH
물리 스위치 ON 또는 GND 연결 = GPIO LOW
```

정상 `CTRL` 패킷에서는 사용하기 편하도록 다음과 같이 변환되어 출력됩니다.

```text
스위치 OFF = 0
스위치 ON  = 1
```

### 4.3 Jetson UART

| ESP32-S3 | Jetson | 용도 |
|---|---|---|
| GPIO7 TX | J12 Pin 10 RX | ESP32 → Jetson 데이터 전송 |
| GPIO6 RX | J12 Pin 8 TX | Jetson → ESP32 수신용 |
| GND | J12 Pin 6 GND | 공통 접지 |

ESP32에서 Jetson으로 데이터만 전송할 경우 최소 연결은 다음과 같습니다.

```text
ESP32-S3 GPIO7 TX -> Jetson J12 Pin 10 RX
ESP32-S3 GND      -> Jetson J12 Pin 6 GND
```

ESP32의 TX는 Jetson의 RX에 연결해야 하며, 두 장치의 GND는 반드시 공통으로 연결해야 합니다.

---

## 5. 출력 패킷

### 5.1 패킷 순서

```text
CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill
```

각 패킷의 끝에는 줄바꿈 문자 `\n`이 추가됩니다.

### 5.2 출력 예시

```text
CTRL,1523,642,-35,120,8,1,1,0,0
```

### 5.3 필드 설명

| 인덱스 | 필드 | 범위 | 설명 |
|---:|---|---:|---|
| 0 | `CTRL` | 문자열 | 조종기 패킷 식별자 |
| 1 | `seq` | 0 이상 정수 | 전송할 때마다 1씩 증가하는 패킷 번호 |
| 2 | `throttle` | 0~1000 | 스로틀 명령 |
| 3 | `roll` | -1000~1000 | Roll 명령 |
| 4 | `pitch` | -1000~1000 | Pitch 명령 |
| 5 | `yaw` | -1000~1000 | Yaw 명령 |
| 6 | `fire` | 0 또는 1 | 발사 명령 |
| 7 | `mode` | 0 또는 1 | 0: Manual, 1: Auto |
| 8 | `eland` | 0 또는 1 | 비상착륙 스위치 |
| 9 | `kill` | 0 또는 1 | Kill 스위치 |

Jetson 수신 프로그램은 한 줄씩 읽은 뒤 쉼표로 분리하고, 필드 개수가 10개인지 확인하여 파싱하는 것을 권장합니다.

---

## 6. 스위치 동작

### Fire

Fire 스위치가 한 번 ON으로 인식되면 `fire=1`로 고정됩니다.

```text
초기 상태       : fire = 0
Fire 한 번 작동 : fire = 1
스위치를 OFF    : fire = 1 유지
```

Fire 값은 ESP32를 재부팅하거나 전원을 다시 인가해야 `0`으로 초기화됩니다.

### Mode, Emergency Landing, Kill

나머지 세 스위치는 래치나 소프트웨어 토글 없이 실제 ON/OFF 상태를 그대로 전송합니다.

```text
스위치 OFF = 0
스위치 ON  = 1
```

| 스위치 | OFF | ON |
|---|---:|---:|
| Mode | Manual = 0 | Auto = 1 |
| Emergency Landing | 0 | 1 |
| Kill | 0 | 1 |

모든 스위치에는 30 ms 디바운싱이 적용됩니다.

---

## 7. 짐벌 출력 범위

| 축 | 출력 범위 | 중앙 또는 최저 |
|---|---:|---:|
| Throttle | 0~1000 | 최저 0 |
| Roll | -1000~1000 | 중앙 0 |
| Pitch | -1000~1000 | 중앙 0 |
| Yaw | -1000~1000 | 중앙 0 |

Roll, Pitch, Yaw에는 중앙 데드밴드가 적용되어 절댓값이 30보다 작으면 `0`으로 출력됩니다.

```cpp
const int CENTER_DEADBAND = 30;
```

---

## 8. 적용된 보정값

현재 펌웨어에는 실제 조종기 출력 결과를 바탕으로 계산한 다음 보정값이 적용되어 있습니다.

| 축 | `raw_min` | `raw_mid` | `raw_max` | `invert` |
|---|---:|---:|---:|---|
| Roll | 525 | 2048 | 3800 | `false` |
| Pitch | 300 | 1931 | 3758 | `true` |
| Throttle | 307 | 사용하지 않음 | 3562 | `false` |
| Yaw | 347 | 1917 | 3800 | `false` |

현재 보정값은 확인된 정규화 출력값을 역산하여 설정한 값입니다. 짐벌이나 ESP32 보드를 교체하거나 배선을 변경한 경우에는 RAW ADC 값을 다시 측정하여 보정하는 것이 좋습니다.

---

## 9. 필터 설정

짐벌 입력에는 1차 저역통과필터가 적용됩니다.

```cpp
filtered[k] =
    LPF_ALPHA * filtered[k-1]
    + (1.0f - LPF_ALPHA) * raw[k];
```

현재 설정은 다음과 같습니다.

```cpp
const float LPF_ALPHA = 0.82f;
```

- 값을 크게 하면 출력이 더 부드러워지지만 반응이 느려집니다.
- 값을 작게 하면 반응은 빨라지지만 ADC 노이즈가 더 크게 나타납니다.
- 현재 `0.82`는 100 Hz 조종기 입력에서 부드러움과 응답 속도를 절충한 값입니다.

---

## 10. 통신 설정

```cpp
const uint32_t SERIAL_BAUD = 115200;
const uint32_t SAMPLE_PERIOD_US = 10000;
```

| 항목 | 설정 |
|---|---|
| UART 속도 | 115200 baud |
| 데이터 비트 | 8 bit |
| 패리티 | 없음 |
| 정지 비트 | 1 bit |
| 패킷 주기 | 10 ms |
| 전송 주파수 | 100 Hz |

ESP32-S3의 USB Serial과 Jetson UART는 서로 다른 통신 채널입니다.

- `Serial`: Arduino IDE Serial Monitor용
- `JetsonSerial`: GPIO6/7을 사용하는 Jetson UART용

따라서 Jetson 통신 코드를 주석 처리할 필요가 없습니다.

```cpp
JetsonSerial.begin(...);
JetsonSerial.write(...);
```

위 부분을 주석 처리하면 Jetson으로 데이터가 전송되지 않습니다.

---

## 11. 디버그 설정

### 정상 운용

```cpp
const bool RAW_DEBUG_MODE = false;
const bool ECHO_PACKET_TO_USB = false;
```

- Jetson으로 정상 패킷 전송
- USB Serial Monitor에는 패킷을 반복 출력하지 않음

### Jetson 전송과 USB 모니터 동시 확인

```cpp
const bool RAW_DEBUG_MODE = false;
const bool ECHO_PACKET_TO_USB = true;
```

- Jetson으로 정상 전송
- 동일한 `CTRL` 패킷을 Arduino Serial Monitor에도 출력

### ADC 보정 모드

```cpp
const bool RAW_DEBUG_MODE = true;
```

이 모드에서는 Jetson으로 `CTRL` 패킷을 보내지 않고 USB Serial Monitor에 RAW ADC 값을 출력합니다.

```text
RAW,roll,pitch,throttle,yaw,kill,eland,mode,fire
```

RAW 모드의 스위치 값은 GPIO 전압 상태를 직접 출력하므로 정상 패킷과 반대입니다.

```text
RAW 스위치 값 1 = 안 눌림 또는 OFF
RAW 스위치 값 0 = 눌림 또는 ON
```

보정이 끝나면 반드시 다음과 같이 되돌립니다.

```cpp
const bool RAW_DEBUG_MODE = false;
```

---

## 12. Arduino IDE 업로드

1. ESP32-S3를 USB로 PC에 연결합니다.
2. Arduino IDE에서 `controller.ino`를 엽니다.
3. 보드를 `ESP32S3 Dev Module`로 선택합니다.
4. ESP32가 연결된 COM 포트를 선택합니다.
5. 코드를 컴파일하고 업로드합니다.
6. Serial Monitor의 속도를 `115200 baud`로 설정합니다.
7. 정상 패킷이 출력되는지 확인합니다.

정상 출력 예시는 다음과 같습니다.

```text
CTRL,0,0,0,0,0,0,0,0,0
CTRL,1,0,0,0,0,0,0,0,0
CTRL,2,15,-8,4,0,0,0,0,0
```

---

## 13. Jetson 수신 시 확인 사항

Jetson에서는 다음 조건을 확인하는 것이 좋습니다.

1. 한 줄이 `CTRL,`로 시작하는지 확인
2. 쉼표로 분리했을 때 필드가 10개인지 확인
3. `seq`가 정상적으로 증가하는지 확인
4. 각 축이 정의된 범위 안에 있는지 확인
5. 마지막 정상 패킷 수신 후 일정 시간이 지나면 통신 끊김으로 판단

안전을 위해 Jetson 수신 프로그램에는 통신 타임아웃을 구현하는 것을 권장합니다. 예를 들어 100~500 ms 동안 정상 패킷이 수신되지 않으면 스로틀을 0으로 만들고 안전 상태로 전환할 수 있습니다.

---

## 14. 점검 기준

### 중앙 또는 기본 위치

```text
Throttle ≈ 0
Roll     ≈ 0
Pitch    ≈ 0
Yaw      ≈ 0
```

### 짐벌 끝 위치

```text
Throttle : 약 0~1000
Roll     : 약 -1000~1000
Pitch    : 약 -1000~1000
Yaw      : 약 -1000~1000
```

### 스위치

```text
Fire  : 한 번 ON 후 계속 1
Mode  : 실제 ON/OFF 상태
E-Land: 실제 ON/OFF 상태
Kill  : 실제 ON/OFF 상태
```

중앙에서 Roll, Pitch, Yaw가 약 ±10~20 정도 흔들리는 것은 일반적인 ADC 노이즈 범위입니다. 중앙에서 ±30 이상 지속적으로 벗어나면 `raw_mid`를 다시 보정합니다.

---

## 15. 주의 사항

- ESP32-S3 ADC 핀에 5 V를 입력하지 마십시오.
- ESP32와 Jetson의 GND를 반드시 공통으로 연결하십시오.
- UART는 TX와 RX를 교차 연결하십시오.
- Fire는 한 번 활성화되면 ESP32 재부팅 전까지 해제되지 않습니다.
- 실제 장비 연결 전에는 프로펠러나 위험한 구동부를 분리한 상태에서 테스트하십시오.
- Jetson 측에서 패킷 순서와 필드 개수를 동일하게 적용해야 합니다.
- `RAW_DEBUG_MODE=true`이면 Jetson으로 정상 조종 패킷이 전송되지 않습니다.

---

## 16. 파일 구성

```text
ESP32_S3_Controller/
├── controller.ino
├── README.md
└── docs/
    └── controller_coordinate_system.png
```
