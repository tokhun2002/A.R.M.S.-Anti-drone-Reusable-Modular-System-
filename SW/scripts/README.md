# CRSF UART 송신 테스트 스크립트

Jetson Orin Nano 40핀 UART → **RadioMaster Ranger Micro (ELRS TX 모듈)** 경로로
CRSF RC 프레임이 제대로 나가는지 ROS 스택 없이 검증하기 위한 스크립트다.
(PX4 쪽에는 ELRS RX 가 물려 있어서, Jetson 이 사실상 조종기 역할을 한다.)

| 파일            | 역할                                                     |
| --------------- | -------------------------------------------------------- |
| `crsf.py`       | CRSF 프레임 인코더/디코더 모듈 (직접 실행하지 않음)      |
| `crsf_send.py`  | UART 로 CRSF RC 프레임 송신                              |
| `crsf_dump.py`  | CRSF 프레임 수신/디코드 — 루프백 자체 검증용             |

프레임 포맷·CRC·스케일링은 실기체 코드
(`arms_ws/src/arms_control/src/crsf_output.cpp`)와 **바이트 단위로 동일**하게 맞춰져 있다.
채널 맵도 `arms_control_node.cpp` 와 같다:

```
CH1 roll   CH2 pitch   CH3 throttle   CH4 yaw
CH5 arm    CH6 land    CH7 kill       CH8 launch/fire
```

## ⚠️ 안전

- **첫 테스트는 반드시 프로펠러를 제거한 상태에서.** 이 스크립트는 arm 스위치(CH5)를
  올릴 수 있고, FC 설정에 따라 모터가 즉시 돌 수 있다.
- 스로틀(CH3)은 기본적으로 **항상 `CRSF_MIN` 으로 강제**된다.
  `--allow-throttle` 을 명시적으로 주지 않는 한 절대 올라가지 않는다.
- Ctrl-C 로 중단하면 종료 전에 중립 failsafe 프레임을 잠깐 흘려보낸다.

## 사전 준비

```bash
pip3 install pyserial
sudo usermod -aG dialout $USER    # 재로그인 필요
```

`/dev/ttyTHS1` 을 시리얼 콘솔이 점유하고 있으면 해제:

```bash
sudo systemctl disable --now nvgetty
```

## 배선

Jetson Orin Nano 40핀 헤더 기준:

| Jetson 핀        | 연결 대상                |
| ---------------- | ------------------------ |
| 핀 8 `UART1_TXD` | 모듈 CRSF 입력 (RX)      |
| 핀 10 `UART1_RXD`| (송신 전용 테스트에선 미사용 — 루프백 때만 핀8과 직결) |
| 핀 6 `GND`       | 모듈 GND                 |

**모듈 전원은 별도 BEC 에서 뽑을 것.** 출력을 올린 ELRS TX 모듈은 순간 전류가 커서
Jetson 5V 레일에서 직접 급전하면 보드가 불안정해질 수 있다.
정확한 입력 전압/전류 범위는 Ranger Micro 자체 스펙을 확인한다.

UART 신호 레벨은 양쪽 다 3.3V 라 레벨 변환은 필요 없다.

## 1단계 — 루프백 검증 (ELRS 하드웨어 없이)

먼저 **Jetson UART 가 요청한 baud 로 실제로 정확히 클럭하는지**부터 분리해서 확인한다.
핀 8(TXD) ↔ 핀 10(RXD) 를 점퍼로 직결하고 터미널 두 개에서:

```bash
python3 crsf_dump.py --baud 420000
```

```bash
python3 crsf_send.py --mode sweep --baud 420000
```

- 디코드된 `roll`/`pitch` 값이 사인파처럼 움직이고 **`err=0`** 이면 그 baud 는 신뢰 가능
- CRC 실패가 계속 섞이면 Tegra UART 분주비 반올림으로 실제 baud 가 어긋난 것 →
  다른 baud 로 재시도

`--baud 460800` 으로도 똑같이 돌려서 **두 값 모두 기록**해 둔다.

### baud 실험 결과 기록

| baud   | 루프백 CRC 에러 | 모듈 인식 | 비고 |
| ------ | --------------- | --------- | ---- |
| 420000 |                 |           | ELRS CRSF 표준 |
| 460800 |                 |           | 현재 `crsf_output.cpp` 가 쓰는 값 |

> 이 표를 채운 뒤 `crsf_output.cpp:26-29` 의 `B460800` 을 맞는 값으로 고치는 게 후속 작업.

## 2단계 — 모듈 연결 테스트

프로펠러를 제거한 상태에서:

```bash
# 모듈 전원 인가 / RX 바인딩 확인 (전 채널 중립)
python3 crsf_send.py --mode hold

# roll/pitch 사인파 — QGC RC 캘리브레이션 화면에서 채널이 움직이는지 확인
python3 crsf_send.py --mode sweep

# CH5 를 3초 간격 토글 — FC 가 aux 채널 변화를 보는지 확인
python3 crsf_send.py --mode arm
```

송신 중 1초마다 실제 달성 프레임레이트와 B/s 가 찍힌다.
요청한 `--rate` 보다 확연히 낮게 나오거나 `write timeout` 이 뜨면
UART 가 그 baud 를 못 따라가고 있는 것이다.

## 주요 옵션

| 인자               | 기본값          | 설명                                   |
| ------------------ | --------------- | -------------------------------------- |
| `--port`           | `/dev/ttyTHS1`  | UART 장치 (Orin Nano 40핀 = UART1)     |
| `--baud`           | `420000`        | ELRS CRSF 표준                         |
| `--rate`           | `100`           | 프레임 송신 주기 [Hz]                  |
| `--mode`           | `hold`          | `hold` / `sweep` / `arm`               |
| `--duration`       | `0`             | 송신 시간 [s], 0 = 무한                |
| `--allow-throttle` | (off)           | 스로틀 강제 MIN 해제 — **위험**        |
| `--throttle`       | `0.0`           | `--allow-throttle` 일 때의 스로틀 0..1 |

## 참고

- 실기체 실행 시 `arms_control` 의 `crsf.port` 파라미터가 기본값
  `/tmp/crsf_tx`(SITL 용 PTY)이므로, 실제 UART 를 쓰려면 `/dev/ttyTHS1` 로 넘겨줘야 한다.
- `crsf_dump.py` 는 ELRS 텔레메트리를 파싱하지 않는다. 순수하게 자기가 보낸 프레임을
  되받는 자체 루프백 검증용이다.
