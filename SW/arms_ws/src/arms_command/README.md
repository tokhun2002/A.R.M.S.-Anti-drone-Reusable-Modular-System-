# arms_command UART Controller Input Node

ESP32-S3에서 짐벌 4축과 스위치 4개의 값을 읽은 뒤 UART로 전송하고, Jetson Orin Nano Super에서 해당 값을 수신하여 ROS2 `sensor_msgs/msg/Joy` 메시지로 변환하는 노드이다.

`arms_command` 패키지에 통합되어 있으며, 최종 출력 토픽은 `/joy`이다.

```text
짐벌 4축 + 스위치 4개
          ↓
       ESP32-S3
          ↓ UART
Jetson Orin Nano Super
          ↓
sensor_msgs/msg/Joy
          ↓
        /joy
```

---

## 주요 기능

ESP32-S3는 다음 작업을 담당한다.

- 짐벌 아날로그 값 읽기
- 짐벌 값 보정 및 정규화
- 1차 저역통과필터 적용
- 중앙 Deadband 적용
- 스위치 디바운싱
- Fire 스위치 latch 처리
- UART 패킷 생성 및 전송

Jetson Orin Nano Super는 다음 작업을 담당한다.

- UART 데이터 수신
- 패킷 형식 검사
- 값 범위 검사
- ESP32 값을 ROS2 Joy 값으로 변환
- `/joy` 토픽 발행
- UART 통신 끊김 감지 및 failsafe 적용

Jetson에서는 더 이상 ADS1115와 GPIO를 직접 읽지 않는다.

---

## 사용 부품

### Jetson

- Jetson Orin Nano Super

### ESP32

- ESP32-S3 N16R8

### 짐벌 2개

- 링크: https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=99100&category=026002008005#review
- Self-centering 짐벌: Roll, Pitch
- Throttle 모드 짐벌: Throttle, Yaw

### 스위치 4개

- 링크: https://www.devicemart.co.kr/goods/view?no=1065126&srsltid=AfmBOooxtzKZoxFYRMIDeEqnLqjBQQcpw4nxR1Z-YsNGoK2pu-5FIOxv

스위치 역할:

```text
스위치 1: Kill
스위치 2: Emergency Landing
스위치 3: Auto / Manual Mode
스위치 4: Fire
```

Kill, Emergency Landing, Mode 스위치는 현재 ON/OFF 상태를 그대로 전송한다.

```text
OFF = 0
ON  = 1
```

Fire 스위치는 한 번 눌리면 `1`로 고정되며, ESP32를 재부팅하기 전까지 유지된다.

---

## 전체 시스템 구조

```text
짐벌 4축
- Roll
- Pitch
- Throttle
- Yaw

스위치 4개
- Kill
- Emergency Landing
- Auto / Manual Mode
- Fire
          ↓
       ESP32-S3
- ADC 입력 처리
- 축 보정
- LPF
- Deadband
- 스위치 디바운싱
- Fire latch
          ↓
UART 115200 bps
          ↓
Jetson Orin Nano Super
/dev/ttyTHS1
          ↓
arms_command_hw_node
          ↓
sensor_msgs/msg/Joy
          ↓
/joy
```

---

## UART 배선

ESP32에서 Jetson으로 단방향 전송만 사용하는 경우 다음 두 가닥을 연결한다.

```text
ESP32-S3 GPIO7 TX → Jetson J12 물리 핀 10 UART1_RXD
ESP32-S3 GND      → Jetson J12 물리 핀 6 GND
```

UART는 송신 핀과 수신 핀을 교차 연결해야 한다.

```text
ESP32 TX → Jetson RX
```

현재 Jetson에서 ESP32로 데이터를 보내지 않으므로 다음 연결은 필요하지 않다.

```text
Jetson TX → ESP32 RX
```

ESP32와 Jetson은 반드시 GND를 공통으로 연결해야 한다.

UART 신호는 3.3V TTL을 사용한다. Jetson의 5V 핀이나 ESP32의 5V 신호를 UART RX 핀에 연결하면 안 된다.

---

## ESP32 UART 패킷

ESP32는 100 Hz 주기로 다음 형식의 문자열을 전송한다.

```text
CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill
```

패킷 예시:

```text
CTRL,1523,642,-35,120,8,1,1,0,0
```

각 필드의 의미:

```text
CTRL       : 패킷 식별 문자열
seq        : 전송 순번
throttle   : 0 ~ 1000
roll       : -1000 ~ 1000
pitch      : -1000 ~ 1000
yaw        : -1000 ~ 1000
fire       : 0 또는 1
mode       : 0 또는 1
eland      : 0 또는 1
kill       : 0 또는 1
```

ESP32 패킷 끝에는 줄바꿈 문자 `\n`이 포함된다.

---

## ROS2 Joy 메시지 매핑

Jetson은 ESP32 패킷을 다음과 같이 `sensor_msgs/msg/Joy` 메시지로 변환한다.

### 축 매핑

```text
axes[0] = Roll
axes[1] = Pitch
axes[2] = Throttle
axes[3] = Yaw
```

Roll, Pitch, Yaw 변환:

```text
ESP32 -1000 → ROS2 -1.0
ESP32     0 → ROS2  0.0
ESP32  1000 → ROS2  1.0
```

Throttle 변환:

```text
ESP32    0 → ROS2 -1.0
ESP32  500 → ROS2  0.0
ESP32 1000 → ROS2  1.0
```

### 버튼 매핑

```text
buttons[0] = Kill
buttons[1] = Emergency Landing
buttons[2] = Auto / Manual Mode
buttons[3] = Fire
```

ESP32 패킷에서는 버튼 순서가 다음과 같다.

```text
fire, mode, eland, kill
```

Jetson ROS2 노드에서 기존 제어 인터페이스에 맞게 다음 순서로 재배치한다.

```text
kill, eland, mode, fire
```

---

## ROS2 출력 토픽

기본 출력 토픽:

```text
/joy
```

메시지 타입:

```text
sensor_msgs/msg/Joy
```

정상 출력 예시:

```yaml
axes:
- 0.12
- -0.35
- 0.48
- -0.02
buttons:
- 0
- 1
- 0
- 0
```

---

## UART 설정

설정 파일 위치:

```text
arms_command/config/command_params.yaml
```

기본 설정:

```yaml
arms_command_hw_node:
  ros__parameters:
    topic_name: /joy

    serial.device: /dev/ttyTHS1
    serial.baud: 115200
    serial.poll_rate_hz: 200.0
    serial.max_line_length: 128

    failsafe.timeout_ms: 300
    failsafe.publish_rate_hz: 20.0

    log.status_rate_hz: 1.0
```

주요 설정:

```text
serial.device
Jetson UART 장치 경로

serial.baud
ESP32와 동일한 UART 통신 속도

serial.poll_rate_hz
Jetson이 UART 수신 버퍼를 확인하는 주기

failsafe.timeout_ms
정상 패킷이 들어오지 않았다고 판단하는 시간

topic_name
Joy 메시지를 발행할 ROS2 토픽
```

---

## UART 장치 확인

Jetson에서 UART 장치를 확인한다.

```bash
ls -l /dev/ttyTHS*
```

기본적으로 다음 장치를 사용한다.

```text
/dev/ttyTHS1
```

실제 장치가 `/dev/ttyTHS0`으로 표시된다면 `command_params.yaml`을 다음과 같이 수정한다.

```yaml
serial.device: /dev/ttyTHS0
```

---

## UART 권한 설정

현재 사용자가 `dialout` 그룹에 포함되어 있는지 확인한다.

```bash
groups
```

`dialout`이 없다면 다음 명령을 실행한다.

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

재부팅 후 다시 로그인해야 권한이 적용된다.

---

## UART 원본 데이터 확인

ROS2 노드를 실행하기 전에 ESP32 패킷이 Jetson까지 정상적으로 들어오는지 확인한다.

```bash
sudo stty -F /dev/ttyTHS1 115200 raw -echo
sudo timeout 3 cat /dev/ttyTHS1
```

정상 출력 예시:

```text
CTRL,120,532,-35,120,8,0,1,0,0
CTRL,121,540,-42,125,10,0,1,0,0
CTRL,122,548,-45,128,12,0,1,0,0
```

아무것도 출력되지 않는다면 다음 항목을 확인한다.

```text
ESP32 GPIO7 TX와 Jetson J12 핀 10 연결
ESP32와 Jetson GND 공통 연결
UART 장치 경로
ESP32 통신속도 115200
ESP32 RAW_DEBUG_MODE가 false인지 확인
```

ESP32 코드에서 다음 설정이어야 한다.

```cpp
const bool RAW_DEBUG_MODE = false;
```

`RAW_DEBUG_MODE`가 `true`이면 ESP32가 Jetson UART로 `CTRL` 패킷을 전송하지 않는다.

UART 원본 데이터 확인이 끝난 뒤 `cat` 명령이 계속 실행 중이라면 `Ctrl+C`로 종료한다.

---

## Failsafe

Jetson이 설정된 시간 동안 정상 ESP32 패킷을 받지 못하면 failsafe가 활성화된다.

기본 통신 끊김 판단 시간:

```text
300 ms
```

Failsafe 상태에서는 다음 Joy 메시지를 발행한다.

```text
axes[0] =  0.0
axes[1] =  0.0
axes[2] = -1.0
axes[3] =  0.0

buttons[0] = 1
buttons[1] = 0
buttons[2] = 0
buttons[3] = 0
```

의미:

```text
Roll 중앙
Pitch 중앙
Throttle 최저
Yaw 중앙
Kill 활성화
Fire 비활성화
```

정상 UART 패킷이 다시 들어오면 failsafe가 해제된다.

---

## 필요한 ROS2 패키지

이 노드는 다음 ROS2 패키지를 사용한다.

```text
rclcpp
sensor_msgs
ament_cmake
ament_cmake_python
```

기존 ADS1115와 Jetson GPIO를 사용하지 않으므로 다음 패키지는 더 이상 필수가 아니다.

```text
libgpiod-dev
gpiod
i2c-tools
```

---

## 빌드

ROS2 워크스페이스로 이동한다.

```bash
cd ~/SW/arms_ws
```

ROS2 환경을 불러온다.

```bash
source /opt/ros/humble/setup.bash
```

`arms_command` 패키지를 빌드한다.

```bash
colcon build --symlink-install --packages-select arms_command
```

빌드된 환경을 적용한다.

```bash
source install/setup.bash
```

기존 ADS1115/GPIO 버전에서 변경한 뒤 빌드 오류가 발생하면 기존 빌드 결과를 삭제하고 다시 빌드한다.

```bash
rm -rf build/arms_command
rm -rf install/arms_command

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arms_command
source install/setup.bash
```

---

## 실행

실기체 UART 입력 노드 실행:

```bash
ros2 launch arms_command command.launch.py
```

실행 노드:

```text
arms_command_hw_node
```

실제 내부 동작은 ESP32 UART 수신 및 ROS2 Joy 변환이다.

---

## 토픽 확인

다른 터미널을 열고 ROS2 환경을 적용한다.

```bash
cd ~/SW/arms_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

`/joy` 토픽 내용을 확인한다.

```bash
ros2 topic echo /joy
```

짐벌을 움직였을 때 `axes` 값이 변하고, 스위치를 조작했을 때 `buttons` 값이 변하면 정상이다.

토픽 발행 주기 확인:

```bash
ros2 topic hz /joy
```

ESP32가 100 Hz로 전송하므로 정상 상태에서는 약 100 Hz로 출력된다.

---

## 파일 구성

실기체 UART 수신 코드:

```text
arms_command/src/arms_command_uart_node.cpp
```

UART 설정 파일:

```text
arms_command/config/command_params.yaml
```

실기체 실행 launch 파일:

```text
arms_command/launch/command.launch.py
```

빌드 설정:

```text
arms_command/CMakeLists.txt
```

---

## 최종 데이터 흐름

```text
짐벌 및 스위치 조작
        ↓
ESP32-S3에서 값 읽기
        ↓
보정, 필터링, 디바운싱
        ↓
CTRL UART 패킷 전송
        ↓
Jetson /dev/ttyTHS1 수신
        ↓
arms_command_uart_node.cpp
        ↓
sensor_msgs/msg/Joy 변환
        ↓
/joy 토픽 발행
```