#!/usr/bin/env python3
"""
crsf_dump.py — CRSF 프레임 수신/디코드 (루프백 자체 검증용)

ELRS 하드웨어를 물리기 *전에*, Jetson UART 가 요청한 baud 로 실제로 정확히
클럭하는지를 분리해서 확인하기 위한 스크립트다.

  Jetson 40핀 헤더에서 핀8(UART1_TXD) ↔ 핀10(UART1_RXD) 를 점퍼로 직결하고,

  터미널 A:  python3 crsf_dump.py  --baud 420000
  터미널 B:  python3 crsf_send.py  --baud 420000 --mode sweep

디코드된 채널값이 송신값과 일치하고 CRC 에러가 0 이면 그 baud 는 신뢰 가능.
CRC 에러가 계속 잡히면 Tegra UART 분주비 반올림으로 실제 baud 가 어긋난 것이므로
다른 baud(460800 등)로 재시도한다.

(주의: ELRS 텔레메트리 파싱이 아니다. 순수하게 자기가 보낸 걸 되받는 자체 루프백.)
"""

import argparse
import sys
import time

import crsf

try:
    import serial
except ImportError:
    sys.exit("pyserial 없음:  pip3 install pyserial")


def main() -> None:
    p = argparse.ArgumentParser(
        description="CRSF 프레임 디코드 (루프백 검증)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--port", default="/dev/ttyTHS1", help="UART 장치")
    p.add_argument("--baud", type=int, default=420000)
    p.add_argument("--interval", type=float, default=1.0, help="채널값 출력 주기 [s]")
    p.add_argument("--channels", type=int, default=8, help="출력할 채널 개수")
    args = p.parse_args()

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    except (serial.SerialException, ValueError) as e:
        sys.exit(f"포트 열기 실패 ({args.port} @ {args.baud}): {e}")

    print(f"수신 대기: {args.port} @ {args.baud} baud  (Ctrl-C 로 중단)\n")

    buf = bytearray()
    good = 0
    crc_err = 0
    report_good = 0
    report_err = 0
    last_channels = None
    t0 = time.monotonic()
    next_report = t0 + args.interval

    try:
        while True:
            chunk = ser.read(512)
            if chunk:
                buf.extend(chunk)

            # 버퍼에서 프레임 추출. 실패하면 1바이트씩 밀며 재동기화한다.
            while len(buf) >= crsf.FRAME_LEN:
                if buf[0] != crsf.CRSF_SYNC:
                    del buf[0]
                    continue
                channels = crsf.decode_rc_frame(bytes(buf[:crsf.FRAME_LEN]))
                if channels is None:
                    # sync 는 맞았지만 길이/타입/CRC 불일치 → 잘못된 경계이거나 비트 깨짐
                    crc_err += 1
                    report_err += 1
                    del buf[0]
                    continue
                good += 1
                report_good += 1
                last_channels = channels
                del buf[:crsf.FRAME_LEN]

            now = time.monotonic()
            if now >= next_report:
                span = now - (next_report - args.interval)
                elapsed = now - t0
                # 이번 구간에 실제로 들어온 프레임이 없으면 채널값을 절대 찍지 않는다.
                # (이전 값을 계속 보여주면 계속 수신 중인 것처럼 오해하게 된다)
                if report_good == 0:
                    note = f"  (CRC/동기 실패 {report_err})" if report_err else ""
                    print(f"[{elapsed:6.1f}s]    0.0 Hz  ---- 수신 없음 ----{note}")
                else:
                    vals = " ".join(
                        f"{crsf.CH_NAMES[i]}={last_channels[i]:4d}"
                        for i in range(min(args.channels, 16))
                    )
                    print(f"[{elapsed:6.1f}s] {report_good/span:6.1f} Hz  "
                          f"err={report_err:<4d} | {vals}")
                report_good = 0
                report_err = 0
                next_report = now + args.interval

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        total = good + crc_err
        rate = (crc_err / total * 100.0) if total else 0.0
        print(f"\n종료: 정상 {good} 프레임 / 실패 {crc_err} ({rate:.2f}%)")
        if crc_err and good:
            print("실패가 섞여 나오면 해당 baud 에서 UART 타이밍이 어긋난 것 — 다른 baud 시도")
        elif not good:
            print("한 프레임도 못 받음 — 배선(핀8↔핀10) / baud / 송신측 실행 여부 확인")


if __name__ == "__main__":
    main()
