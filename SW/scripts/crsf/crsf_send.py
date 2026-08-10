#!/usr/bin/env python3
"""
crsf_send.py — Jetson UART → ELRS TX 모듈 CRSF 송신 테스트

ROS 스택 없이 "UART로 CRSF 프레임이 제대로 나가는가"만 따로 검증하기 위한 스크립트.
배선/baud/프레임 포맷을 벤치에서 확인한 뒤 실기체에 올린다.

채널 맵은 arms_control_node.cpp 의 CRSF 출력과 동일:
  CH1 roll  CH2 pitch  CH3 throttle  CH4 yaw
  CH5 arm   CH6 land   CH7 kill      CH8 launch/fire

사용:
  python3 crsf_send.py                          # /dev/ttyTHS1, 420000, hold 모드
  python3 crsf_send.py --mode sweep             # roll/pitch 사인파
  python3 crsf_send.py --baud 460800 --mode arm # baud 비교 실험

!! 안전: 기본적으로 스로틀(CH3)은 항상 CRSF_MIN 으로 강제된다.
   --allow-throttle 을 명시적으로 주지 않는 한 절대 스로틀이 올라가지 않는다.
"""

import argparse
import math
import sys
import time

import crsf

try:
    import serial
except ImportError:
    sys.exit("pyserial 없음:  pip3 install pyserial")


# baud 탐색 후보 — 흔한 것부터.
#   400000 : ELRS/EdgeTX 계열에서 가장 흔함
#   420000 : CRSF 표준으로 널리 인용되는 값
#   460800 : 현재 crsf_output.cpp 가 쓰는 값 (표준 baud 중 420000 에 가장 가까움)
SWEEP_BAUDS = [400000, 420000, 416666, 460800, 115200, 921600]

BANNER = """
╔══════════════════════════════════════════════════════════════╗
║  CRSF UART 송신 테스트                                       ║
║                                                              ║
║  프로펠러를 제거한 상태에서 테스트할 것.                     ║
║  이 스크립트는 arm 스위치(CH5)를 올릴 수 있으며,             ║
║  FC 설정에 따라 모터가 즉시 회전할 수 있다.                  ║
╚══════════════════════════════════════════════════════════════╝
"""


def open_port(port: str, baud: int) -> "serial.Serial":
    """
    시리얼 포트 열기.
    420000 같은 비표준 baud 는 pyserial 이 Linux 에서 TCSETS2 + BOTHER ioctl 로
    처리한다. 드라이버가 거부하면 여기서 예외가 난다.
    """
    try:
        return serial.Serial(port=port, baudrate=baud, timeout=0, write_timeout=1.0)
    except (OSError, ValueError) as e:
        # SerialException 은 OSError 하위지만, 드라이버가 비표준 baud 를 거부할 때는
        # ioctl 이 맨 OSError 를 던지므로 둘 다 받아야 한다.
        sys.exit(
            f"포트 열기 실패 ({port} @ {baud}): {e}\n"
            f"  - 권한 문제면:  sudo usermod -aG dialout $USER  (재로그인 필요)\n"
            f"  - 시리얼 콘솔이 점유 중이면:  sudo systemctl disable --now nvgetty\n"
            f"  - 이 baud 를 드라이버가 거부하는 것이면:  --baud-sweep 으로 후보 탐색"
        )


def make_channels(mode: str, t: float, allow_throttle: bool) -> list:
    """경과 시간 t[s] 에 대한 16채널 값을 만든다."""
    ch = crsf.neutral_channels()

    if mode == "hold":
        pass  # 중립 유지 — 모듈 전원/바인딩 확인용 baseline

    elif mode == "sweep":
        # 4초 주기 사인파. pitch 는 90도 위상차를 줘서 두 채널을 구분해 볼 수 있게 함
        phase = 2.0 * math.pi * t / 4.0
        ch[crsf.CH_ROLL] = crsf.norm_to_crsf(math.sin(phase))
        ch[crsf.CH_PITCH] = crsf.norm_to_crsf(math.cos(phase))

    elif mode == "arm":
        # 3초 간격 CH5 토글
        ch[crsf.CH_ARM] = crsf.CRSF_MAX if int(t / 3.0) % 2 else crsf.CRSF_MIN

    if not allow_throttle:
        ch[crsf.CH_THROTTLE] = crsf.CRSF_MIN

    return ch


def send_failsafe(ser, rate_hz: float) -> None:
    """종료 전 중립 프레임을 잠깐 흘려서 모듈/수신기가 마지막에 안전한 상태를 보게 한다."""
    frame = crsf.build_rc_frame(crsf.neutral_channels())
    period = 1.0 / rate_hz
    for _ in range(20):
        try:
            ser.write(frame)
            ser.flush()
        except serial.SerialException:
            return
        time.sleep(period)


def baud_sweep(args) -> None:
    """
    후보 baud 를 하나씩 돌아가며 중립 프레임을 송신한다.
    모듈 LED 가 "CRSF 없음"(느린 명멸) → "정상"(고정) 으로 바뀌는 순간의 baud 가 정답.
    """
    print("=" * 62)
    print("  baud 탐색 — 모듈 LED 를 계속 지켜볼 것")
    print(f"  각 baud 당 {args.sweep_secs:g}초씩 중립 프레임 송신")
    print("  LED 가 느린 명멸을 멈추고 고정되는 구간의 baud 가 정답")
    print("=" * 62 + "\n")

    frame = crsf.build_rc_frame(crsf.neutral_channels())
    period = 1.0 / args.rate

    for baud in SWEEP_BAUDS:
        try:
            ser = serial.Serial(port=args.port, baudrate=baud,
                                timeout=0, write_timeout=1.0)
        except (OSError, ValueError) as e:
            # 드라이버가 그 baud 를 거부하면 건너뛰고 계속 — 탐색이 여기서 죽으면 안 된다
            print(f"  {baud:>7d} baud … 열기 실패 (드라이버 미지원): {e}")
            continue

        print(f"→ {baud:>7d} baud 송신 중 … ", end="", flush=True)
        t_end = time.monotonic() + args.sweep_secs
        sent = 0
        try:
            while time.monotonic() < t_end:
                ser.write(frame)
                sent += 1
                time.sleep(period)
            ser.flush()
            print(f"{sent} 프레임 전송 완료")
        except serial.SerialTimeoutException:
            print(f"write timeout ({sent} 프레임에서 막힘)")
        except serial.SerialException as e:
            print(f"쓰기 실패: {e}")
        except KeyboardInterrupt:
            ser.close()
            print("\n\n중단됨.")
            return
        finally:
            ser.close()

    print("\n탐색 끝. LED 가 바뀐 baud 를 --baud 로 지정해서 다시 확인:")
    print(f"  python3 crsf_send.py --baud <값> --mode hold")
    print("어느 구간에서도 변화가 없었으면 baud 가 아니라 배선/핀 문제일 가능성이 높다.")


def main() -> None:
    p = argparse.ArgumentParser(
        description="Jetson UART → ELRS 모듈 CRSF 송신 테스트",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--port", default="/dev/ttyTHS1", help="UART 장치 (Orin Nano 40핀 = UART1)")
    p.add_argument("--baud", type=int, default=420000, help="ELRS CRSF 표준은 420000")
    p.add_argument("--rate", type=float, default=100.0, help="프레임 송신 주기 [Hz]")
    p.add_argument("--mode", choices=["hold", "sweep", "arm"], default="hold")
    p.add_argument("--duration", type=float, default=0.0, help="송신 시간 [s], 0 = 무한")
    p.add_argument("--allow-throttle", action="store_true",
                   help="스로틀 강제 MIN 해제 (위험 — 프로펠러 제거 확인)")
    p.add_argument("--throttle", type=float, default=0.0,
                   help="--allow-throttle 일 때의 스로틀 0..1")
    p.add_argument("--baud-sweep", action="store_true",
                   help="후보 baud 를 차례로 송신 — 모듈 LED 가 언제 바뀌는지 보고 판별")
    p.add_argument("--sweep-secs", type=float, default=6.0,
                   help="--baud-sweep 에서 baud 하나당 송신 시간 [s]")
    args = p.parse_args()

    if args.rate <= 0:
        sys.exit("--rate 는 0보다 커야 함")

    print(BANNER)
    if args.allow_throttle:
        print(f"!! --allow-throttle 활성: 스로틀 {args.throttle:.2f} 로 송신됨\n")

    if args.baud_sweep:
        baud_sweep(args)
        return

    ser = open_port(args.port, args.baud)
    print(f"열림: {args.port} @ {args.baud} baud")
    print(f"모드: {args.mode}   송신 주기: {args.rate:g} Hz   (Ctrl-C 로 중단)\n")

    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_deadline = t0
    next_report = t0 + 1.0
    frames = 0
    total_bytes = 0
    report_frames = 0
    report_bytes = 0

    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0

            if args.duration > 0 and elapsed >= args.duration:
                break

            ch = make_channels(args.mode, elapsed, args.allow_throttle)
            if args.allow_throttle:
                ch[crsf.CH_THROTTLE] = crsf.thr_to_crsf(args.throttle)

            frame = crsf.build_rc_frame(ch)
            try:
                n = ser.write(frame)
            except serial.SerialTimeoutException:
                # write_timeout 초과 = UART 가 요청한 baud 를 못 따라가고 있다는 신호
                print("!! write timeout — baud 대비 송신 주기가 너무 높거나 포트가 막힘")
                break
            except serial.SerialException as e:
                print(f"!! 쓰기 실패: {e}")
                break

            frames += 1
            report_frames += 1
            total_bytes += n or 0
            report_bytes += n or 0

            # 1초마다 실제 달성 레이트 보고 — 요청 baud 를 못 따라가면 여기서 드러난다
            if now >= next_report:
                span = now - (next_report - 1.0)
                ch_preview = " ".join(
                    f"{crsf.CH_NAMES[i]}={ch[i]}" for i in range(4)
                )
                print(f"[{elapsed:6.1f}s] {report_frames/span:6.1f} Hz  "
                      f"{report_bytes/span:8.0f} B/s   {ch_preview}")
                report_frames = 0
                report_bytes = 0
                next_report = now + 1.0

            # 드리프트 없는 고정 주기 대기
            next_deadline += period
            sleep_for = next_deadline - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # 밀렸으면 따라잡기를 포기하고 기준선을 현재로 리셋
                next_deadline = time.monotonic()

    except KeyboardInterrupt:
        print("\n중단 — failsafe 프레임 송신 중...")

    finally:
        # 평균 레이트는 failsafe 송신 구간을 빼고 계산해야 실제 성능이 나온다
        elapsed = time.monotonic() - t0
        send_failsafe(ser, args.rate)
        ser.close()
        print(f"\n종료: {frames} 프레임 / {total_bytes} 바이트 / {elapsed:.1f}s "
              f"(평균 {frames/elapsed if elapsed else 0:.1f} Hz)")


if __name__ == "__main__":
    main()
