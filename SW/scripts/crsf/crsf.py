#!/usr/bin/env python3
"""
crsf.py — CRSF (Crossfire) RC 프레임 인코더/디코더

arms_control 의 C++ 구현과 와이어 포맷이 반드시 일치해야 테스트 의미가 있으므로,
아래 로직은 그대로 옮긴 것이다:
  - 프레임 빌드 / CRC / 11bit 패킹 : arms_control/src/crsf_output.cpp
  - 디코드(CRC 검증 포함)           : arms_control/arms_control/sitl_bridge_node.py

프레임 구조 (RC channels packed, 총 26바이트):
  [0]     0xC8   sync / FC address
  [1]     24     length = type(1) + payload(22) + crc(1)
  [2]     0x16   frame type = RC channels packed
  [3..24] payload — 16채널 × 11bit 리틀엔디안 비트패킹
  [25]    CRC8/DVB-S2 (poly 0xD5), [2]..[24] 구간에 대해 계산
"""

CRSF_SYNC = 0xC8
CRSF_TYPE_RC = 0x16

CRSF_MIN = 172
CRSF_CENTER = 992
CRSF_MAX = 1811

FRAME_LEN = 26  # sync + length + (type + payload + crc)

# 채널 인덱스 — arms_control_node.cpp 의 CRSF 출력 맵과 동일
CH_ROLL = 0
CH_PITCH = 1
CH_THROTTLE = 2
CH_YAW = 3
CH_ARM = 4
CH_LAND = 5
CH_KILL = 6
CH_FIRE = 7

CH_NAMES = [
    "roll", "pitch", "throttle", "yaw",
    "arm", "land", "kill", "fire",
    "ch9", "ch10", "ch11", "ch12",
    "ch13", "ch14", "ch15", "ch16",
]


def crc8_dvb_s2(data: bytes) -> int:
    """CRC8/DVB-S2 (poly 0xD5). crsf_output.cpp 의 crc8_dvb_s2 와 동일."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def norm_to_crsf(v: float) -> int:
    """정규화 값 (-1..1) → CRSF 단위. 0 이 CRSF_CENTER 로 간다."""
    v = max(-1.0, min(1.0, v))
    if v >= 0.0:
        return int(CRSF_CENTER + v * (CRSF_MAX - CRSF_CENTER))
    return int(CRSF_CENTER + v * (CRSF_CENTER - CRSF_MIN))


def thr_to_crsf(v: float) -> int:
    """스로틀 (0..1) → CRSF 단위. 0 이 CRSF_MIN 으로 간다."""
    v = max(0.0, min(1.0, v))
    return int(CRSF_MIN + v * (CRSF_MAX - CRSF_MIN))


def crsf_to_us(v: int) -> int:
    """CRSF 단위 (172–1811) → 마이크로초 (1000–2000). 사람이 읽기 좋게 표시할 때만 사용."""
    return int(1000 + (v - CRSF_MIN) / (CRSF_MAX - CRSF_MIN) * 1000)


def neutral_channels() -> list:
    """안전한 기본 채널 상태 — 스틱 중립, 스로틀 최소, 모든 스위치 LOW."""
    ch = [CRSF_MIN] * 16
    ch[CH_ROLL] = CRSF_CENTER
    ch[CH_PITCH] = CRSF_CENTER
    ch[CH_YAW] = CRSF_CENTER
    ch[CH_THROTTLE] = CRSF_MIN
    return ch


def build_rc_frame(channels) -> bytes:
    """16채널 (CRSF 단위) → 26바이트 CRSF RC 프레임."""
    if len(channels) != 16:
        raise ValueError(f"need exactly 16 channels, got {len(channels)}")

    f = bytearray(FRAME_LEN)
    f[0] = CRSF_SYNC
    f[1] = 24
    f[2] = CRSF_TYPE_RC

    # 16 × 11bit 를 f[3..24] 에 리틀엔디안 비트 순서로 패킹
    bit = 0
    for value in channels:
        val = max(CRSF_MIN, min(CRSF_MAX, int(value)))
        for b in range(11):
            if val & (1 << b):
                f[3 + bit // 8] |= 1 << (bit % 8)
            bit += 1

    # CRC 는 type + payload 구간 (f[2]..f[24] = 23바이트)
    f[25] = crc8_dvb_s2(bytes(f[2:25]))
    return bytes(f)


def decode_rc_frame(buf: bytes):
    """
    26바이트 CRSF RC 프레임 → 16채널 리스트.
    RC 프레임이 아니거나 CRC 불일치면 None 을 돌려준다.
    """
    if len(buf) < FRAME_LEN:
        return None
    if buf[0] != CRSF_SYNC or buf[1] != 24 or buf[2] != CRSF_TYPE_RC:
        return None
    if crc8_dvb_s2(buf[2:25]) != buf[25]:
        return None

    channels = []
    bit = 0
    for _ in range(16):
        val = 0
        for b in range(11):
            if buf[3 + bit // 8] & (1 << (bit % 8)):
                val |= 1 << b
            bit += 1
        channels.append(val)
    return channels
