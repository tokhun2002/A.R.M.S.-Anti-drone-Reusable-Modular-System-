# ESP32-S3 조종기 송신기

ESP32-S3에서 짐벌 4축과 스위치 4개의 입력을 읽고, USB CDC를 통해 Jetson으로 100 Hz 전송하는 조종기 펌웨어입니다.

## 주요 기능

- 조종축: `Throttle`, `Roll`, `Pitch`, `Yaw`
- 스위치: `Fire`, `Mode`, `Emergency Landing`, `Kill`
- 모든 스위치는 실제 ON/OFF 상태를 `0` 또는 `1`로 전송
- 짐벌 입력에 1차 LPF와 중앙 데드밴드 적용
- USB CDC 장치 `/dev/ttyACM0` 사용

## 좌표계

<img width="886" height="666" alt="좌표계" src="https://github.com/user-attachments/assets/ce088d08-abd4-4d44-a44c-a36500f112f2" />

- X축: 전후 방향
- Y축: 좌우 방향
- Z축: 수직 위쪽
- Roll: X축 회전
- Pitch: Y축 회전
- Yaw: Z축 회전

축 방향이 반대면 해당 `AxisConfig`의 `invert` 값을 변경합니다.

```cpp
false  // 현재 방향 유지
true   // 출력 부호 반전
```

## 핀 연결

### 짐벌

| 기능 | ESP32-S3 핀 |
|---|---:|
| Roll | GPIO1 |
| Pitch | GPIO2 |
| Throttle | GPIO4 |
| Yaw | GPIO5 |

```text
짐벌 VCC -> ESP32-S3 3V3
짐벌 GND -> ESP32-S3 GND
```

### 스위치

| 기능 | ESP32-S3 핀 |
|---|---:|
| Kill | GPIO15 |
| Emergency Landing | GPIO16 |
| Mode | GPIO17 |
| Fire | GPIO18 |

각 스위치는 해당 GPIO와 GND 사이에 연결합니다.

```text
OFF = 0
ON  = 1
```

ESP32-S3 ADC 입력에는 5 V를 연결하지 않습니다.

## Jetson 연결

GPIO UART는 사용하지 않고 USB로 연결합니다.

```text
ESP32-S3 USB-C 포트 -> Jetson USB 포트
```

데이터 통신 가능한 USB 케이블을 사용해야 합니다.

## ESP32 업로드 설정

Arduino IDE:

```text
Board           : ESP32S3 Dev Module
USB CDC On Boot : Enabled
```

정상 운용 설정:

```cpp
static constexpr bool RAW_DEBUG_MODE = false;
```

## Jetson 최초 설정

USB 연결 후 장치를 확인합니다.

```bash
ls -l /dev/ttyACM*
```

예상 장치:

```text
/dev/ttyACM0
```

시리얼 권한을 추가합니다.

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

직접 수신 테스트가 필요하면 다음 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install -y python3-serial
```

수신 테스트:

```bash
python3 - <<'PY'
import serial

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as port:
    port.dtr = True
    port.rts = False

    while True:
        line = port.readline()
        if line:
            print(line.decode(errors='replace').strip())
PY
```

종료는 `Ctrl+C`입니다.

Arduino Serial Monitor, Python 테스트, ROS2 노드는 `/dev/ttyACM0`을 동시에 사용할 수 없습니다.

## 전송 패킷

```text
CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill
```

예시:

```text
CTRL,1523,642,-35,120,8,1,1,0,0
```

| 필드 | 범위 |
|---|---:|
| `seq` | 0 이상 |
| `throttle` | 0~1000 |
| `roll` | -1000~1000 |
| `pitch` | -1000~1000 |
| `yaw` | -1000~1000 |
| `fire` | 0 또는 1 |
| `mode` | 0 또는 1 |
| `eland` | 0 또는 1 |
| `kill` | 0 또는 1 |

모든 스위치는 현재 물리 상태를 그대로 전송합니다.

## ROS2 실행

`arms_command/config/command_params.yaml`:

```yaml
serial.device: /dev/ttyACM0
serial.baud: 115200
```

저장소 최상위 폴더에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to arms_command
source install/setup.bash
ros2 launch arms_command command.launch.py
```

새 터미널에서 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /joy
```

전송 주기 확인:

```bash
ros2 topic hz /joy
```

ROS2 출력 순서:

```text
axes[0] = roll
axes[1] = pitch
axes[2] = throttle
axes[3] = yaw

buttons[0] = kill
buttons[1] = eland
buttons[2] = mode
buttons[3] = fire
```

## RAW 보정 모드

ADC 값을 다시 측정할 때만 다음 값을 `true`로 변경합니다.

```cpp
static constexpr bool RAW_DEBUG_MODE = true;
```

출력 형식:

```text
RAW,roll,pitch,throttle,yaw,kill,eland,mode,fire
```

보정 후 반드시 `false`로 되돌립니다.