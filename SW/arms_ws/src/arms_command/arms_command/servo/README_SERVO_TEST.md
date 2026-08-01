# MG996R 서보 테스트

동작은 다음과 같다.

- `set_servo(1)` → 90도
- `set_servo(2)` → 180도
- ARM 스위치 0 → 90도
- ARM 스위치 1 → 180도

## 1. 배선

- 서보 신호선 → Jetson 물리 15번 핀(GPIO12/PWM)
- 서보 전원선 → 외부 5~6V BEC
- 서보 GND → BEC GND
- BEC GND → Jetson GND(예: 물리 14번 핀)

> MG996R 전원을 Jetson 5V 핀에 직접 연결하지 않는다. 테스트 전 프로펠러를 분리한다.

## 2. PWM 활성화

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

40핀 헤더 수동 설정에서 15번 핀의 `PWM/PWM1`을 활성화하고 저장한 뒤 재부팅한다.

Jetson.GPIO가 없으면 설치한다.

```bash
sudo pip3 install Jetson.GPIO
```

## 3. 빌드

```bash
cd ~/ARMS/SW/arms_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arms_command
source install/setup.bash
```

## 4. 서보만 테스트

90도:

```bash
python3 -c 'import time; from arms_command.servo.servo_motor import set_servo; set_servo(1); time.sleep(2)'
```

180도:

```bash
python3 -c 'import time; from arms_command.servo.servo_motor import set_servo; set_servo(2); time.sleep(2)'
```

## 5. ROS 메시지로 테스트

첫 번째 터미널:

```bash
source /opt/ros/humble/setup.bash
source ~/ARMS/SW/arms_ws/install/setup.bash
ros2 run arms_command servo_switch_test_node
```

두 번째 터미널에서 90도 명령:

```bash
ros2 topic pub --once /arms/command sensor_msgs/msg/Joy "{buttons: [0, 0, 0, 0]}"
```

180도 명령:

```bash
ros2 topic pub --once /arms/command sensor_msgs/msg/Joy "{buttons: [0, 1, 0, 0]}"
```

## 6. 실제 ARM 스위치로 테스트

조종기 노드가 실행 중인 상태에서 다음 노드를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ARMS/SW/arms_ws/install/setup.bash
ros2 run arms_command servo_switch_test_node
```

ARM 스위치를 움직여 다음 동작을 확인한다.

- 스위치 0 → 90도
- 스위치 1 → 180도

다른 코드에서는 다음 함수만 사용한다.

```python
from arms_command.servo.servo_motor import set_servo

set_servo(1)
set_servo(2)
```

서보가 움직이지 않으면 PWM 활성화, 재부팅, 외부 전원, 공통 GND, `buttons[1]` 값을 확인한다.
