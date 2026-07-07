#!/usr/bin/env python3
"""
arms_comm_sitl_node — SITL 통신 노드 (pymavlink, Stabilized mode)

/arms/ctrl_cmd  → RC_CHANNELS_OVERRIDE (50Hz) → PX4 Stabilized
/arms/mission_state → ARM / RTL / payload actuator 명령
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

from arms_msgs.msg import CtrlCmd, MissionState

# PX4 custom_mode constants for set_mode
PX4_BASE_MODE_CUSTOM = 1
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5


class ArmsSITLCommNode(Node):
    def __init__(self):
        super().__init__("arms_comm_sitl_node")

        self.declare_parameter("connection", "udpin:0.0.0.0:14540")
        self.declare_parameter("max_angle_deg", 35.0)
        self.declare_parameter("send_rate_hz", 50.0)

        self._conn_str = self.get_parameter("connection").value
        self._max_angle = self.get_parameter("max_angle_deg").value
        self._send_rate = self.get_parameter("send_rate_hz").value

        # Last received control command
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._thrust = 0.0
        self._armed = False
        self._last_state = "IDLE"
        self._rtl_sent = False
        self._lock = threading.Lock()

        self._mav = None
        self._connected = False

        if not _PYMAVLINK:
            self.get_logger().error("pymavlink not installed. Run: pip3 install pymavlink")
        else:
            threading.Thread(target=self._connect_loop, daemon=True).start()

        self.create_subscription(CtrlCmd, "/arms/ctrl_cmd", self._cb_ctrl, 10)
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)

        self.create_timer(1.0 / self._send_rate, self._send_override)
        self.get_logger().info(f"arms_comm_sitl_node ready (conn={self._conn_str})")

    # ------------------------------------------------------------------
    def _connect_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to PX4: {self._conn_str} ...")
                mav = mavutil.mavlink_connection(self._conn_str)
                mav.wait_heartbeat(timeout=10)
                self._mav = mav
                self._connected = True
                self.get_logger().info(
                    f"Connected. system={mav.target_system} component={mav.target_component}")
                self._setup_stabilized()
                break
            except Exception as e:
                self.get_logger().warn(f"Connection failed: {e}. Retrying in 3s...")
                time.sleep(3.0)

    def _setup_stabilized(self):
        """ARM and switch to Stabilized mode."""
        time.sleep(3.0)  # EKF convergence

        self.get_logger().info("Arming...")
        self._send_arm(True)
        time.sleep(1.0)

        self.get_logger().info("Setting Stabilized mode...")
        self._send_set_mode(PX4_CUSTOM_MAIN_MODE_STABILIZED)
        time.sleep(0.5)

        with self._lock:
            self._armed = True
        self.get_logger().info("SITL: armed and Stabilized mode set.")

    # ------------------------------------------------------------------
    def _cb_ctrl(self, msg: CtrlCmd):
        with self._lock:
            self._roll_deg = msg.roll_deg
            self._pitch_deg = msg.pitch_deg
            self._thrust = msg.thrust

    def _cb_state(self, msg: MissionState):
        state = msg.state
        with self._lock:
            prev = self._last_state
            self._last_state = state

        if state == "RTL" and prev != "RTL":
            self.get_logger().info("Mission state RTL → switching to RTL mode")
            self._send_set_mode_rtl()
        elif state == "FIRE" and prev != "FIRE":
            self.get_logger().info("Mission state FIRE → sending payload actuator")
            self._send_payload()

    # ------------------------------------------------------------------
    def _send_override(self):
        if not self._connected or self._mav is None:
            return

        with self._lock:
            roll = self._roll_deg
            pitch = self._pitch_deg
            thrust = self._thrust
            state = self._last_state

        # RC_CHANNELS_OVERRIDE: 1000–2000 range, center 1500
        max_a = self._max_angle if self._max_angle > 0 else 35.0

        ch1 = int(1500 + (roll / max_a) * 500)   # roll
        ch2 = int(1500 + (pitch / max_a) * 500)  # pitch
        ch3 = int(1000 + thrust * 1000)            # throttle
        ch4 = 1500                                  # yaw (hold)
        ch5 = 1811 if state not in ("IDLE",) else 988  # arm switch

        ch1 = max(1000, min(2000, ch1))
        ch2 = max(1000, min(2000, ch2))
        ch3 = max(1000, min(2000, ch3))

        try:
            self._mav.mav.rc_channels_override_send(
                self._mav.target_system,
                self._mav.target_component,
                ch1, ch2, ch3, ch4, ch5,
                65535, 65535, 65535,  # ch6–ch8 passthrough
            )
        except Exception as e:
            self.get_logger().warn(f"RC override send failed: {e}", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    def _send_arm(self, arm: bool):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,  # param1: 1=arm, 0=disarm
            0, 0, 0, 0, 0, 0,
        )

    def _send_set_mode(self, main_mode: int, sub_mode: int = 0):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(PX4_BASE_MODE_CUSTOM),  # param1: base_mode
            float(main_mode),              # param2: custom_main_mode (직접 값)
            float(sub_mode),               # param3: custom_sub_mode (직접 값)
            0, 0, 0, 0,
        )

    def _send_set_mode_rtl(self):
        self._send_set_mode(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL)

    def _send_payload(self):
        if self._mav is None:
            return
        # MAV_CMD_DO_SET_ACTUATOR: actuator 0 (payload servo) to max
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR,
            0,
            1.0,  # actuator index 0 → max
            0, 0, 0, 0, 0, 0,
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
