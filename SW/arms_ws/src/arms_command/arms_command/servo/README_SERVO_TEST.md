# MG996R 서보 테스트

Jetson Orin Nano의 하드웨어 PWM을 사용하여 MG996R 서보모터를 제어한다.

동작은 다음과 같다.

* `set_servo(1)` → 90도
* `set_servo(2)` → 180도
* ARM 스위치 0 → 90도
* ARM 스위치 1 → 180도

---

## 1. 배선

* 서보 신호선 → Jetson 40핀 헤더 물리 15번 핀

  * `GPIO12`
  * `PWM1`
* 서보 전원선 → 외부 5~6V BEC
* 서보 GND → BEC GND
* BEC GND → Jetson GND

  * 예: Jetson 물리 14번 핀

> MG996R 전원을 Jetson의 5V 핀에 직접 연결하지 않는다.

> Jetson GND와 외부 BEC GND는 반드시 공통으로 연결한다.

> 실제 장비에서 테스트하기 전에 프로펠러를 분리한다.

---

## 2. PWM 활성화

다음 명령으로 Jetson-IO를 실행한다.

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

다음 순서로 설정한다.

```text
Configure Jetson 40-pin Header
→ Configure header pins manually
→ 물리 15번 핀의 PWM/PWM1 활성화
→ Save pin changes
→ Save and reboot
```

설정을 저장한 뒤 Jetson을 재부팅한다.

```bash
sudo reboot
```

---

## 3. Jetson.GPIO 설치 및 모델 인식 설정

### 3-1. Jetson.GPIO 설치

```bash
sudo python3 -m pip install \
  --upgrade \
  --ignore-installed \
  --no-cache-dir \
  Jetson.GPIO
```

설치 상태를 확인한다.

```bash
python3 -c "import Jetson.GPIO as GPIO; print(GPIO.__file__); print(GPIO.VERSION)"
```

### 3-2. Jetson Orin Nano 모델 지정

다음과 같은 오류가 발생할 수 있다.

```text
Exception: Could not determine Jetson model
```

이 경우 Jetson 모델을 환경 변수로 직접 지정한다.

```bash
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
```

정상적으로 인식되는지 확인한다.

```bash
python3 -c "import Jetson.GPIO as GPIO; print(GPIO.VERSION); print(GPIO.JETSON_INFO)"
```

정상적으로 실행되면 로그인할 때마다 적용되도록 `~/.bashrc`에 추가한다.

```bash
grep -qxF \
  'export JETSON_MODEL_NAME=JETSON_ORIN_NANO' \
  ~/.bashrc || \
  echo 'export JETSON_MODEL_NAME=JETSON_ORIN_NANO' >> ~/.bashrc
```

현재 터미널에도 적용한다.

```bash
source ~/.bashrc
```

---

## 4. GPIO 권한 설정

GPIO 권한 오류가 발생하면 다음 설정을 적용한다.

```bash
sudo groupadd -f -r gpio
sudo usermod -aG gpio "$USER"
```

설치된 `99-gpio.rules` 파일을 찾는다.

```bash
GPIO_RULE=$(find \
  /usr/local/lib \
  /usr/lib \
  -path "*/Jetson/GPIO/99-gpio.rules" \
  2>/dev/null | head -n 1)

echo "$GPIO_RULE"
```

찾은 규칙을 등록한다.

```bash
sudo cp "$GPIO_RULE" /etc/udev/rules.d/99-gpio.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

사용자 그룹 변경 사항을 적용하기 위해 재부팅한다.

```bash
sudo reboot
```

---

## 5. ROS2 패키지 빌드

프로젝트의 실제 ROS2 작업공간으로 이동한다.

```bash
cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System--main/SW/arms_ws
```

ROS2 Humble 환경을 불러온다.

```bash
source /opt/ros/humble/setup.bash
```

`arms_command`가 사용하는 메시지 패키지까지 함께 빌드한다.

```bash
colcon build \
  --symlink-install \
  --packages-select arms_msgs arms_command
```

빌드된 작업공간 환경을 불러온다.

```bash
source install/setup.bash
```

> `source install/setup.bash`와 다음 명령어를 붙여 쓰지 않는다.

잘못된 예:

```bash
source install/setup.bashpython3 -c '...'
```

올바른 예:

```bash
source install/setup.bash
python3 -c '...'
```

한 줄로 실행할 때는 `&&`를 사용한다.

```bash
source install/setup.bash && python3 -c '...'
```

---

## 6. 서보 단독 테스트

먼저 작업공간 환경을 불러온다.

```bash
cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System--main/SW/arms_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 6-1. 90도 테스트

```bash
python3 -c 'import time; from arms_command.servo.servo_motor import set_servo; set_servo(1); time.sleep(2)'
```

### 6-2. 180도 테스트

```bash
python3 -c 'import time; from arms_command.servo.servo_motor import set_servo; set_servo(2); time.sleep(2)'
```

---

## 7. ROS 메시지로 테스트

### 첫 번째 터미널

```bash
cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System--main/SW/arms_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arms_command servo_switch_test_node
```

### 두 번째 터미널

두 번째 터미널에서도 ROS2 환경을 불러온다.

```bash
cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System--main/SW/arms_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 7-1. 90도 명령

`buttons[1] = 0`을 전송한다.

```bash
ros2 topic pub \
  --once \
  /arms/command \
  sensor_msgs/msg/Joy \
  "{buttons: [0, 0, 0, 0]}"
```

### 7-2. 180도 명령

`buttons[1] = 1`을 전송한다.

```bash
ros2 topic pub \
  --once \
  /arms/command \
  sensor_msgs/msg/Joy \
  "{buttons: [0, 1, 0, 0]}"
```

---

## 8. 실제 ARM 스위치로 테스트

조종기 입력 노드가 실행 중인 상태에서 다음 노드를 실행한다.

```bash
cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System--main/SW/arms_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arms_command servo_switch_test_node
```

ARM 스위치를 움직여 다음 동작을 확인한다.

* ARM 스위치 0 → 90도
* ARM 스위치 1 → 180도

현재 서보 테스트 노드는 `/arms/command` 메시지의 다음 값을 사용한다.

```text
buttons[1]
```

---

## 9. 다른 Python 코드에서 사용

다른 Python 코드에서는 다음 함수를 불러와 사용한다.

```python
from arms_command.servo.servo_motor import set_servo
```

90도로 이동:

```python
set_servo(1)
```

180도로 이동:

```python
set_servo(2)
```

---

## 10. 문제 해결

### `Could not determine Jetson model`

다음을 실행한다.

```bash
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
```

확인:

```bash
python3 -c "import Jetson.GPIO as GPIO; print(GPIO.JETSON_INFO)"
```

### `install/setup.bashpython3: No such file or directory`

`source` 명령과 `python3` 명령 사이가 분리되지 않은 것이다.

```bash
source install/setup.bash
python3 -c '...'
```

또는:

```bash
source install/setup.bash && python3 -c '...'
```

### `arms_msgs/package.sh`를 찾을 수 없음

잘못된 작업공간에서 빌드했거나 `arms_msgs`를 빌드하지 않은 것이다.

```bash
cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System--main/SW/arms_ws

source /opt/ros/humble/setup.bash

rm -rf build install log

colcon build \
  --symlink-install \
  --packages-select arms_msgs arms_command

source install/setup.bash
```

### 서보가 움직이지 않음

다음 항목을 확인한다.

1. Jetson-IO에서 물리 15번 핀의 `PWM/PWM1` 활성화
2. 설정 저장 후 Jetson 재부팅
3. `JETSON_MODEL_NAME=JETSON_ORIN_NANO` 설정
4. 외부 5~6V BEC 전원 연결
5. Jetson GND와 BEC GND 공통 연결
6. 서보 신호선이 물리 15번 핀에 연결되어 있는지 확인
7. ROS 메시지의 `buttons[1]` 값 확인
8. `servo_switch_test_node` 실행 여부 확인

토픽 값을 확인하려면 다음 명령을 사용한다.

```bash
ros2 topic echo /arms/command
```
