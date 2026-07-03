#!/usr/bin/env python3
"""
arms_command_gpio_node.py — 실기체용 커맨드 노드 (GPIO 버튼 입력)

SITL GUI(arms_command_node)와 동일한 토픽 인터페이스:
  publishes:  /arms/launch_cmd  (std_msgs/Empty)
  subscribes: /arms/mission_state (arms_msgs/MissionState)

실기체 GPIO 핀 번호는 파라미터로 설정:
  launch_pin: LAUNCH 버튼 GPIO 핀 번호 (default: 17)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from arms_msgs.msg import MissionState


class GpioCommandNode(Node):
    def __init__(self):
        super().__init__("arms_command_node")

        self.declare_parameter("launch_pin", 17)
        self._launch_pin = self.get_parameter("launch_pin").value

        self._pub_launch = self.create_publisher(Empty, "/arms/launch_cmd", 1)
        self._sub_state = self.create_subscription(
            MissionState, "/arms/mission_state", self._on_state, 10
        )

        self._state = None
        self._setup_gpio()
        self.get_logger().info(f"GPIO command node ready (launch_pin={self._launch_pin})")

    def _setup_gpio(self):
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._launch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                self._launch_pin, GPIO.FALLING,
                callback=self._on_launch_press, bouncetime=300,
            )
            self._gpio = GPIO
        except ImportError:
            self.get_logger().warn("RPi.GPIO not available — GPIO input disabled")
            self._gpio = None

    def _on_launch_press(self, _channel):
        self.get_logger().info("LAUNCH button pressed")
        self._pub_launch.publish(Empty())

    def _on_state(self, msg: MissionState):
        self._state = msg

    def destroy_node(self):
        if self._gpio:
            self._gpio.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpioCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
