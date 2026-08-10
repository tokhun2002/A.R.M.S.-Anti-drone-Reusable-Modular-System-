# ELRS CRSF RX/TX 테스트

이 테스트는 Jetson이 Ranger Micro로 RC 채널을 송신하고, 같은 CRSF 선으로 돌아오는
ELRS 텔레메트리를 수신하는지 확인한다.

정상 동작은 다음과 같다.

- `echo` 증가 → Jetson 핀 8 TX 신호가 핀 10 RX로 수신됨
- `valid` 증가 → Ranger Micro 텔레메트리 프레임 수신 성공
- `type=0x14` → ELRS RSSI, LQ, SNR 수신 성공
- `crc_err=0`, `frame_err=0` → baud와 배선 상태 정상

## 1. 배선

Jetson Orin Nano 40핀 헤더 기준으로 연결한다.

```text
Jetson 핀 10 UART1_TXD ── 2.2kΩ ──┬── Ranger Micro CRSF
                                  └── Jetson 핀 8 UART1_RXD

Jetson 핀 6 GND ───────────────────── Ranger Micro GND
```

- 4.7kΩ 저항은 핀 8 TX 선에만 연결한다.
- 핀 10 RX는 저항 뒤 CRSF 접속점에 연결한다.
- Ranger Micro 전원은 별도 BEC 또는 XT30으로 공급한다.
- Ranger Micro 안테나는 전원을 넣기 전에 장착한다.

> 테스트 전 프로펠러를 제거한다. Ranger 전원을 Jetson 3.3V 또는 5V 핀에서 공급하지 않는다.

## 2. UART 준비

UART 장치를 확인한다.

```bash
ls -l /dev/ttyTHS1
```

시리얼 콘솔이 UART를 점유하지 않도록 해제한다.

```bash
sudo systemctl disable --now nvgetty
```

현재 UART를 사용하는 프로세스가 있는지 확인한다.

```bash
sudo fuser -v /dev/ttyTHS1
```

권한 오류가 발생하면 사용자를 `dialout` 그룹에 추가하고 재로그인한다.

```bash
sudo usermod -aG dialout $USER
```

## 3. 빌드

```bash
cd ~/ARMS/SW/arms_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arms_control
source install/setup.bash
```

## 4. RX/TX 단독 테스트

Ranger Micro와 기체의 ELRS RX가 바인딩된 상태에서 제어 노드만 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ARMS/SW/arms_ws/install/setup.bash
ros2 launch arms_control control.launch.py
```

다른 터미널에서 적용된 UART 설정을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ARMS/SW/arms_ws/install/setup.bash
ros2 param get /arms_control_node crsf.port
ros2 param get /arms_control_node crsf.baud
```

정상 설정값은 다음과 같다.

```text
String value is: /dev/ttyTHS1
Integer value is: 400000
```

## 5. 정상 로그 확인

핀 8과 핀 10이 정상 연결되면 `echo`가 계속 증가한다.

```text
CRSF RX: bytes=1560 valid=0 echo=60 crc_err=0 frame_err=0
```

Ranger에서 텔레메트리가 돌아오면 `valid`가 증가하고 새로운 타입이 표시된다.

```text
CRSF telemetry detected: addr=0xEA type=0x14 payload=10
ELRS link: up_rssi=(-70,-72)dBm up_lq=99% up_snr=-3dB down_rssi=-80dBm down_lq=90% down_snr=5dB
```

`Ctrl-C`를 눌러 테스트를 종료한다.

## 6. 로그별 확인 항목

| 로그 상태 | 의미 | 확인할 항목 |
| --- | --- | --- |
| `bytes=0`, `echo=0` | Jetson RX에 아무 신호도 없음 | 핀 10, 공통 GND, UART pinmux, `/dev/ttyTHS1` |
| `echo` 증가, `valid=0` | Jetson 자체 RX/TX는 정상이나 Ranger 응답 없음 | ELRS 바인딩, 기체 전원, FC 텔레메트리, Telemetry Ratio |
| `valid` 증가 | Ranger 텔레메트리 수신 성공 | 정상 |
| `type=0x14` 출력 | ELRS Link Statistics 수신 성공 | RSSI, LQ, SNR 값 확인 |
| `crc_err` 지속 증가 | 수신 파형 또는 속도가 맞지 않음 | 4.7kΩ 접촉, 배선 길이, 400000 baud, 신호 극성 |
| `frame_err` 지속 증가 | 바이트 정렬 또는 노이즈 문제 | GND, 배선 접촉, 다른 프로세스의 UART 점유 |
| `CRSF TX failed` | UART 송신 실패 | 장치 권한, 포트 점유, `/dev/ttyTHS1` 존재 여부 |

## 7. 전체 시스템에서 확인

단독 테스트가 성공한 뒤 전체 실기체 런치를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ARMS/SW/arms_ws/install/setup.bash
ros2 launch arms_bringup arms.launch.py crsf_port:=/dev/ttyTHS1 start_detection:=false
```

터미널에 `CRSF RX`, `CRSF telemetry detected`, `ELRS link` 로그가 계속 출력되면
전체 시스템에서도 CRSF 양방향 통신이 정상이다.

