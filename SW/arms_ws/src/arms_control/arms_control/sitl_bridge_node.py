#!/usr/bin/env python3
"""
sitl_bridge_node — CRSF→MAVLink bridge for SITL

Reads CRSF RC frames from a virtual serial port (socat PTY pair) and converts
them to MAVLink RC_CHANNELS_OVERRIDE toward PX4.

Channel mapping:
  CH1: roll     CH2: pitch    CH3: throttle  CH4: yaw
  CH5: arm sw   CH6: land sw  CH7: kill sw   CH8: launch/fire
"""

import threading
import time

import rclpy
from rclpy.node import Node

try:
    from pymavlink import mavutil
    _PYMAVLINK = True
except ImportError:
    _PYMAVLINK = False

# PX4 mode constants
PX4_BASE_MODE_CUSTOM          = 1
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
PX4_CUSTOM_MAIN_MODE_AUTO     = 4
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6

CRSF_SYNC   = 0xC8
CRSF_TYPE_RC = 0x16
CRSF_MIN     = 172
CRSF_MAX     = 1811
SWITCH_THRESH = 1500  # > threshold → switch is ON

MAV_MODE_FLAG_SAFETY_ARMED = 128
ARM_RETRY_INTERVAL_SEC = 1.0  # resend arm/disarm while desired != actual armed state


def _crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc


def _crsf_to_us(v: int) -> int:
    """CRSF units (172–1811) → microseconds (1000–2000)."""
    return int(1000 + (v - CRSF_MIN) / (CRSF_MAX - CRSF_MIN) * 1000)


def _decode_crsf_frame(fd) -> list | None:
    """
    Read one CRSF RC frame from binary file-like fd.
    Returns list of 16 channel values (CRSF units) or None on error.
    """
    # Scan for sync byte
    while True:
        b = fd.read(1)
        if not b:
            return None
        if b[0] == CRSF_SYNC:
            break

    lb = fd.read(1)
    if not lb:
        return None
    length = lb[0]  # type + payload + crc

    rest = fd.read(length)
    if len(rest) < length:
        return None

    frame_type = rest[0]
    crc_received = rest[-1]
    if frame_type != CRSF_TYPE_RC:
        return None  # skip non-RC frames

    # CRC over type + payload
    expected_crc = _crc8_dvb_s2(rest[:-1])
    if crc_received != expected_crc:
        return None

    payload = rest[1:-1]  # 22 bytes for 16-channel RC frame
    if len(payload) < 22:
        return None

    # Unpack 16 × 11-bit channels (little-endian bit order)
    channels = []
    bit_pos = 0
    for _ in range(16):
        val = 0
        for b in range(11):
            byte_idx = bit_pos // 8
            bit_idx = bit_pos % 8
            if byte_idx < len(payload):
                val |= ((payload[byte_idx] >> bit_idx) & 1) << b
            bit_pos += 1
        channels.append(val)

    return channels


class ArmsSITLCommNode(Node):
    def __init__(self):
        super().__init__("arms_sitl_bridge_node")

        self.declare_parameter("connection", "udpin:0.0.0.0:14540")
        self.declare_parameter("crsf_port", "/tmp/crsf_rx")
        self.declare_parameter("send_rate_hz", 50.0)

        self._conn_str   = self.get_parameter("connection").value
        self._crsf_port  = self.get_parameter("crsf_port").value
        self._send_rate  = self.get_parameter("send_rate_hz").value

        self._channels  = [CRSF_MIN] * 16
        self._channels[2] = CRSF_MIN   # throttle min
        self._prev_ch6  = CRSF_MIN     # land switch
        self._ch5_high  = False        # arm switch, level (not edge) — kept in sync with CH5
        self._land_pending = False

        self._armed = False            # last known FC armed state, from HEARTBEAT
        self._last_arm_send = 0.0      # throttle for arm/disarm resend

        self._mav       = None
        self._connected = False
        self._lock      = threading.Lock()

        if not _PYMAVLINK:
            self.get_logger().error("pymavlink not installed: pip3 install pymavlink")
            return

        threading.Thread(target=self._connect_loop, daemon=True).start()
        threading.Thread(target=self._crsf_read_loop, daemon=True).start()
        threading.Thread(target=self._mavlink_rx_loop, daemon=True).start()

        self.create_timer(1.0 / self._send_rate, self._send_override)
        self.get_logger().info(
            f"sitl_bridge_node ready  conn={self._conn_str}  crsf={self._crsf_port}")

    # ------------------------------------------------------------------
    # MAVLink connection
    # ------------------------------------------------------------------
    def _connect_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to PX4: {self._conn_str} ...")
                mav = mavutil.mavlink_connection(self._conn_str)
                mav.wait_heartbeat(timeout=15)
                self._mav = mav
                self.get_logger().info(
                    f"Connected. system={mav.target_system} "
                    f"component={mav.target_component}")
                self._initial_setup()
                break
            except Exception as e:
                self.get_logger().warn(f"Connection failed: {e}. Retrying in 3s...")
                time.sleep(3.0)

    def _initial_setup(self):
        """Set Stabilized mode after EKF convergence; arming is done via CH5."""
        time.sleep(3.0)
        self.get_logger().info("Setting Stabilized mode...")
        self._send_set_mode(PX4_CUSTOM_MAIN_MODE_STABILIZED)
        time.sleep(0.5)

        self._connected = True
        self.get_logger().info("SITL: Stabilized mode set. Waiting for arm signal (CH5).")

    # ------------------------------------------------------------------
    # MAVLink RX (armed-state tracking via HEARTBEAT)
    # ------------------------------------------------------------------
    def _mavlink_rx_loop(self):
        """Keep _armed in sync with the FC so the retry loop knows when to stop."""
        while rclpy.ok():
            if self._mav is None:
                time.sleep(0.1)
                continue
            try:
                msg = self._mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                if msg is not None:
                    self._armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            except Exception as e:
                self.get_logger().warn(f"MAVLink rx error: {e}", throttle_duration_sec=5.0)
                time.sleep(0.5)

    # ------------------------------------------------------------------
    # CRSF reader
    # ------------------------------------------------------------------
    def _crsf_read_loop(self):
        while rclpy.ok():
            try:
                fd = open(self._crsf_port, 'rb', buffering=0)
                self.get_logger().info(f"CRSF port opened: {self._crsf_port}")
                while rclpy.ok():
                    channels = _decode_crsf_frame(fd)
                    if channels:
                        with self._lock:
                            self._channels = channels
                            self._process_switches(channels)
            except Exception as e:
                self.get_logger().warn(f"CRSF read error: {e}. Retrying in 2s...")
                time.sleep(2.0)

    def _process_switches(self, channels):
        """Track CH5 switch level (arm target) and detect CH6 rising edge (land)."""
        self._ch5_high = channels[4] > SWITCH_THRESH

        ch6 = channels[5]
        if ch6 > SWITCH_THRESH and self._prev_ch6 <= SWITCH_THRESH:
            self._land_pending = True
        self._prev_ch6 = ch6

    # ------------------------------------------------------------------
    # 50 Hz send timer
    # ------------------------------------------------------------------
    def _send_override(self):
        if not self._connected or self._mav is None:
            return

        with self._lock:
            chs = list(self._channels)
            ch5_high = self._ch5_high
            land_req = self._land_pending
            self._land_pending = False

        # Arm/disarm: keep resending while the FC's actual armed state doesn't match
        # CH5 yet. A single MAV_CMD_COMPONENT_ARM_DISARM can be TEMPORARILY_REJECTED
        # (e.g. EKF/position estimate not converged yet right after boot) — without a
        # retry that one lost attempt meant "denied forever" until CH5 toggled again.
        now = time.time()
        if ch5_high != self._armed and (now - self._last_arm_send) >= ARM_RETRY_INTERVAL_SEC:
            self._last_arm_send = now
            self.get_logger().info(
                f"CH5={'ON' if ch5_high else 'OFF'}, armed={self._armed} → "
                f"sending {'ARM' if ch5_high else 'DISARM'}")
            self._send_arm(ch5_high)

        # Handle pending land
        if land_req:
            self.get_logger().info("CH6↑ → LAND mode")
            self._send_set_mode(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND)

        # RC_CHANNELS_OVERRIDE: CRSF → microseconds
        us = [_crsf_to_us(v) for v in chs]
        try:
            self._mav.mav.rc_channels_override_send(
                self._mav.target_system,
                self._mav.target_component,
                us[0], us[1], us[2], us[3],
                us[4], us[5], us[6], us[7],
            )
        except Exception as e:
            self.get_logger().warn(
                f"RC override failed: {e}", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # MAVLink helpers
    # ------------------------------------------------------------------
    def _send_arm(self, arm: bool):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0, 0, 0, 0, 0, 0,
        )

    def _send_set_mode(self, main_mode: int, sub_mode: int = 0):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(PX4_BASE_MODE_CUSTOM),
            float(main_mode),
            float(sub_mode),
            0, 0, 0, 0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmsSITLCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
