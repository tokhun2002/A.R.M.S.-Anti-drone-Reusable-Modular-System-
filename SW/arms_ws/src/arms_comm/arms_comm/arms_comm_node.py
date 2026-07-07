#!/usr/bin/env python3
"""
arms_comm_node — 실기체 통신 노드 (pyserial, CRSF → ELRS → FC, Stabilized mode)

/arms/ctrl_cmd  → CRSF RC_CHANNELS_PACKED 프레임 → serial → ELRS TX → FC
/arms/mission_state → ch5(arm), ch6(mode), ch7(payload) 세팅
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node

try:
    import serial
    _SERIAL = True
except ImportError:
    _SERIAL = False

from arms_msgs.msg import CtrlCmd, MissionState

# CRSF constants
CRSF_SYNC = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_NUM_CHANNELS = 16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MAX = 1811
CRSF_CHANNEL_CENTER = 992

# CRC-8 with polynomial 0xD5 (CRSF standard)
_CRC8_TABLE = bytes(
    (lambda poly=0xD5: [
        v for i in range(256)
        for v in [
            (lambda x=i << 1: (
                (x & 0xFF) ^ poly if x & 0x100 else x & 0xFF
            ))()
        ]
    ])()
)

# Actually compute a proper CRC8 table
def _make_crc8_table(poly=0xD5):
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

_CRC8_D5 = _make_crc8_table(0xD5)


def crsf_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC8_D5[crc ^ b]
    return crc


def pack_crsf_channels(channels: list) -> bytes:
    """Pack 16 channels (each 11-bit) into 22 bytes CRSF payload."""
    assert len(channels) == 16
    bits = 0
    bit_count = 0
    payload = bytearray()
    for ch in channels:
        ch = max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, int(ch)))
        bits |= (ch << bit_count)
        bit_count += 11
        while bit_count >= 8:
            payload.append(bits & 0xFF)
            bits >>= 8
            bit_count -= 8
    if bit_count > 0:
        payload.append(bits & 0xFF)
    return bytes(payload)


def build_crsf_frame(channels: list) -> bytes:
    """Build complete CRSF frame: [sync][len][type][payload][crc]"""
    payload = pack_crsf_channels(channels)
    frame_type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED
    # len = type + payload + crc = 1 + 22 + 1 = 24
    frame_len = 1 + len(payload) + 1
    crc_data = bytes([frame_type]) + payload
    crc = crsf_crc8(crc_data)
    return bytes([CRSF_SYNC, frame_len, frame_type]) + payload + bytes([crc])


def angle_to_crsf(angle_deg: float, max_angle: float) -> int:
    """Map angle [-max_angle, +max_angle] → [172, 1811] with center 992."""
    ratio = angle_deg / max_angle
    ch = CRSF_CHANNEL_CENTER + ratio * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_CENTER)
    return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, int(ch)))


def thrust_to_crsf(thrust: float) -> int:
    """Map thrust [0.0, 1.0] → [172, 1811]."""
    ch = CRSF_CHANNEL_MIN + thrust * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)
    return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, int(ch)))


class ArmsCommNode(Node):
    def __init__(self):
        super().__init__("arms_comm_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 420000)
        self.declare_parameter("max_angle_deg", 35.0)
        self.declare_parameter("send_rate_hz", 50.0)

        self._port = self.get_parameter("serial_port").value
        self._baud = self.get_parameter("baud").value
        self._max_angle = self.get_parameter("max_angle_deg").value
        self._send_rate = self.get_parameter("send_rate_hz").value

        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._thrust = 0.0
        self._last_state = "IDLE"
        self._lock = threading.Lock()

        self._ser = None

        if not _SERIAL:
            self.get_logger().error("pyserial not installed. Run: pip3 install pyserial")
        else:
            threading.Thread(target=self._open_serial, daemon=True).start()

        self.create_subscription(CtrlCmd, "/arms/ctrl_cmd", self._cb_ctrl, 10)
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)

        self.create_timer(1.0 / self._send_rate, self._send_crsf)
        self.get_logger().info(
            f"arms_comm_node ready (port={self._port}, baud={self._baud})")

    # ------------------------------------------------------------------
    def _open_serial(self):
        while rclpy.ok():
            try:
                self._ser = serial.Serial(
                    self._port, self._baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1,
                )
                self.get_logger().info(f"Serial opened: {self._port} @ {self._baud}")
                break
            except Exception as e:
                self.get_logger().warn(
                    f"Cannot open {self._port}: {e}. Retrying in 3s...",
                    throttle_duration_sec=10.0)
                time.sleep(3.0)

    # ------------------------------------------------------------------
    def _cb_ctrl(self, msg: CtrlCmd):
        with self._lock:
            self._roll_deg = msg.roll_deg
            self._pitch_deg = msg.pitch_deg
            self._thrust = msg.thrust

    def _cb_state(self, msg: MissionState):
        with self._lock:
            self._last_state = msg.state

    # ------------------------------------------------------------------
    def _send_crsf(self):
        if self._ser is None or not self._ser.is_open:
            return

        with self._lock:
            roll = self._roll_deg
            pitch = self._pitch_deg
            thrust = self._thrust
            state = self._last_state

        max_a = self._max_angle if self._max_angle > 0 else 35.0

        # Build 16-channel CRSF frame
        channels = [CRSF_CHANNEL_CENTER] * 16

        channels[0] = angle_to_crsf(roll, max_a)    # ch1: roll
        channels[1] = angle_to_crsf(pitch, max_a)   # ch2: pitch
        channels[2] = thrust_to_crsf(thrust)          # ch3: throttle
        channels[3] = CRSF_CHANNEL_CENTER             # ch4: yaw hold

        # ch5: arm switch — disarm only in IDLE
        channels[4] = CRSF_CHANNEL_MAX if state != "IDLE" else CRSF_CHANNEL_MIN

        # ch6: flight mode — RTL or Stabilized
        channels[5] = CRSF_CHANNEL_MAX if state == "RTL" else CRSF_CHANNEL_CENTER

        # ch7: payload trigger
        channels[6] = CRSF_CHANNEL_MAX if state == "FIRE" else CRSF_CHANNEL_MIN

        frame = build_crsf_frame(channels)
        try:
            self._ser.write(frame)
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArmsCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
