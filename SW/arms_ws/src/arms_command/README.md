# arms_command

ESP32-S3 조종기 패킷을 Jetson의 USB CDC 장치에서 읽어 ROS2 `sensor_msgs/msg/Joy`로 변환하는 패키지입니다. 기본 출력 토픽은 `/joy`입니다.

```text
ESP32-S3 USB-C → Jetson USB → /dev/ttyACM0 → arms_command_hw_node → /joy
```

## 패킷 및 ROS2 순서

ESP32 패킷:

```text
CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill
```

ROS2에서도 ESP32와 같은 순서를 사용합니다.

```text
axes[0] = throttle
axes[1] = roll
axes[2] = pitch
axes[3] = yaw

buttons[0] = fire
buttons[1] = mode
buttons[2] = eland
buttons[3] = kill
```

축 변환 범위:

```text
Throttle 0~1000      →  0.0~1.0
Roll/Pitch/Yaw ±1000 → -1.0~1.0
```

통신이 300 ms 이상 끊기면 failsafe로 다음 값을 발행합니다.

```text
axes   = [0.0, 0.0, 0.0, 0.0]
buttons = [0, 0, 0, 1]
```

## Jetson 최초 설정

ESP32-S3를 데이터 통신이 가능한 USB 케이블로 연결한 뒤 장치를 확인합니다.

```bash
ls -l /dev/ttyACM*
```

시리얼 권한이 없다면 다음을 한 번 실행하고 재부팅합니다.

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

직접 수신 테스트에 필요한 패키지:

```bash
sudo apt update
sudo apt install -y python3-serial
```

## 설정

`config/command_params.yaml`:

```yaml
serial.device: /dev/ttyACM0
serial.baud: 115200
serial.poll_rate_hz: 200.0
failsafe.timeout_ms: 300
log.status_rate_hz: 1.0
```

USB 연결 순서에 따라 장치가 `/dev/ttyACM1`로 바뀌면 설정 파일도 동일하게 수정합니다.

## 빌드 및 실행

저장소 최상위 폴더에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to arms_command
source install/setup.bash
ros2 launch arms_command command.launch.py
```

새 터미널에서 출력 확인:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /joy
```

발행 주기 확인:

```bash
ros2 topic hz /joy
```

ESP32가 100 Hz로 전송하므로 정상 상태에서는 약 100 Hz가 나옵니다.

> Arduino Serial Monitor, Python 시리얼 테스트, ROS2 노드는 `/dev/ttyACM0`을 동시에 열 수 없습니다.

## 출력이 느려 보이는 이유

`log.status_rate_hz: 1.0`은 실행 터미널 로그를 1초에 한 번만 출력하도록 제한합니다. 실제 `/joy` 발행 속도와는 다릅니다.

또한 `ros2 topic echo /joy`는 100 Hz 메시지를 YAML 형식으로 계속 출력하므로 터미널 출력이 밀려 실제 제어 입력보다 늦게 보일 수 있습니다. 실제 통신 속도는 다음 명령으로 확인합니다.

```bash
ros2 topic hz /joy
```

ESP32의 LPF도 약간의 반응 지연을 만듭니다. 현재 `LPF_ALPHA=0.82`, 100 Hz 설정에서는 통신 자체보다 필터와 터미널 표시가 체감 지연의 주요 원인입니다.

## 주의사항

- 기존 ROS2 순서인 `[roll, pitch, throttle, yaw]`와 `[kill, eland, mode, fire]`를 사용하던 하위 노드는 새 순서에 맞게 수정해야 합니다.
- 패킷이 수신되지 않으면 다른 프로그램이 `/dev/ttyACM0`을 사용 중인지 확인합니다.

```bash
sudo fuser -v /dev/ttyACM0
```