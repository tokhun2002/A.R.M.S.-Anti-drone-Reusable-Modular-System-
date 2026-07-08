# controller_input_node

Jetson Orin Nano Super에서 조종기 입력을 읽어 ROS2 `/joy` 토픽으로 publish하는 노드이다.  
`arms_command` 패키지에 통합되어 있다.

- ADS1115 A0~A3 → `sensor_msgs/msg/Joy.axes[0..3]`
- GPIO 스위치 4개 → `sensor_msgs/msg/Joy.buttons[0..3]`
- 출력 토픽: `/joy`

---

## 사용 부품

### 짐벌 2개

- 링크: https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=99100&category=026002008005#review

### 스위치 4개

- 링크: https://www.devicemart.co.kr/goods/view?no=1065126&srsltid=AfmBOooxtzKZoxFYRMIDeEqnLqjBQQcpw4nxR1Z-YsNGoK2pu-5FIOxv
- 스위치1: 킬스위치
- 스위치2: 비상착륙
- 스위치3: 자동/수동 비행모드 선택
- 스위치4: LAUNCH (LOCK → TRACK 전환)

```text
OFF / 안 눌림 = 0
ON / 눌림     = 1
```

### ADS1115 ADC

- 링크: https://www.devicemart.co.kr/goods/view?no=1327550

---

## 전체 구조

```text
controller_input_node
    ↓
/dev/i2c-1 에서 ADS1115 A0~A3 읽기
    ↓
GPIO 스위치 4개 읽기
    ↓
sensor_msgs/msg/Joy publish
```

---

## Joy 메시지 매핑

```text
axes[0] = 왼쪽 짐벌 Signal 1
axes[1] = 왼쪽 짐벌 Signal 2
axes[2] = 오른쪽 짐벌 Signal 1
axes[3] = 오른쪽 짐벌 Signal 2

buttons[0] = 킬스위치
buttons[1] = 비상착륙
buttons[2] = 자동/수동 비행모드 선택
buttons[3] = LAUNCH (LOCK → TRACK 전환)
```

짐벌 값은 `-1.0 ~ 1.0` 범위로 정규화된다.

```text
최소 방향 ≈ -1.0
중앙     ≈  0.0
최대 방향 ≈  1.0
```

---

## fake_mode

설정 파일 위치:

```text
arms_command/config/controller_input_params.yaml
```

부품 없이 가상값으로 테스트할 때:

```yaml
fake_mode: true
```

실제 ADS1115와 GPIO를 읽을 때:

```yaml
fake_mode: false
```

`fake_mode: true`에서는 실제 하드웨어 대신 가상의 ADS1115 raw 값을 생성한다.  
짐벌은 3.3V 구동, ADS1115는 ±4.096V 범위로 설정한다고 가정한다.

```text
0.0V   ≈ raw 0
1.65V  ≈ raw 13200
3.3V   ≈ raw 26400
```

---

## ADS1115 설정

기본 설정:

```yaml
i2c:
  device: "/dev/i2c-1"
  address: 72
```

`72`는 10진수 주소이고, 16진수로는 `0x48`이다.  
ADS1115의 `ADDR` 핀을 GND에 연결하면 기본 주소는 `0x48`이다.

I2C 인식 확인:

```bash
i2cdetect -y -r 1
```

정상 연결 시 `0x48` 주소가 보여야 한다.

---

## GPIO 주의사항

Jetson 40핀 기준으로는 29번, 31번, 33번, 35번 핀 등을 사용한다.

하지만 C++ `libgpiod`에서는 40핀 보드 번호가 아니라 **GPIO line offset**을 사용한다.

```text
Jetson 40핀 BOARD pin number ≠ libgpiod GPIO line offset
```

따라서 실제 하드웨어 모드에서는 `gpioinfo`로 line offset을 확인한 뒤 설정 파일에 넣어야 한다.

```bash
gpioinfo
```

설정 예시:

```yaml
gpio:
  chip: "gpiochip0"
  lines: [12, 13, 14, 15]   # [kill, emergency_land, mode, launch]
```

위 숫자는 예시이며, 실제 Jetson에서 확인한 값을 사용해야 한다.

---

## 필요한 패키지 설치

```bash
sudo apt update
sudo apt install -y libgpiod-dev gpiod i2c-tools
sudo usermod -aG i2c,gpio $USER
sudo reboot
```

---

## 빌드

```bash
cd ~/SW/arms_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arms_command
source install/setup.bash
```

---

## 실행

실기체:

```bash
ros2 launch arms_command command.launch.py
```

---

## 토픽 확인

다른 터미널에서 실행한다.

```bash
cd ~/SW/arms_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /joy
```

정상 실행 시 `/joy` 토픽에서 `axes`와 `buttons` 값이 출력된다.

```text
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
