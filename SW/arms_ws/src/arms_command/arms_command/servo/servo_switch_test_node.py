#!/usr/bin/env python3
"""ARM/DISARM 스위치 값을 MG996R 서보 명령으로 변환해 시험한다."""

import rclpy                                                                # ROS 2 Python API를 사용한다.
from rclpy.node import Node                                                 # ROS 2 노드를 구성한다.
from rclpy.qos import qos_profile_sensor_data                               # 조종기 토픽과 같은 Best Effort QoS를 사용한다.
from sensor_msgs.msg import Joy                                             # 조종기 스위치 메시지를 구독한다.

from arms_command.servo.servo_motor import set_servo                        # 단순화한 서보 제어 함수를 사용한다.

COMMAND_TOPIC = "/arms/command"                                            # 실제 조종기 명령 토픽을 구독한다.
ARM_SWITCH_BUTTON_INDEX = 1                                                 # buttons[1]의 ARM/DISARM 값을 사용한다.


class ServoSwitchTestNode(Node):
    # /* ARM/DISARM 스위치와 서보를 연결하는 시험 노드를 만든다. */
    def __init__(self):
        super().__init__("servo_switch_test_node")                          # ROS 2 노드 이름을 지정한다.
        self._last_switch_value = None                                       # 스위치 변화가 있을 때만 서보를 갱신한다.
        self._subscription = self.create_subscription(                       # Joy 메시지 구독을 생성한다.
            Joy,
            COMMAND_TOPIC,
            self._on_command,
            qos_profile_sensor_data,
        )
        self.get_logger().info(                                              # 시험 조건을 시작 로그에 표시한다.
            "서보 테스트 시작: buttons[1] 0 -> 90도, 1 -> 180도"
        )

    # /* 수신한 ARM/DISARM 스위치 값으로 서보를 이동한다. */
    def _on_command(self, message):
        if len(message.buttons) <= ARM_SWITCH_BUTTON_INDEX:                  # ARM 스위치 값이 없는 메시지는 무시한다.
            self.get_logger().warning("/arms/command에 buttons[1] 값이 없음")
            return

        switch_value = 1 if message.buttons[ARM_SWITCH_BUTTON_INDEX] else 0  # 입력을 0 또는 1로 정규화한다.
        if switch_value == self._last_switch_value:                          # 같은 값의 반복 메시지는 처리하지 않는다.
            return

        self._last_switch_value = switch_value                               # 새 스위치 값을 다음 비교용으로 저장한다.
        servo_command = 1 if switch_value == 0 else 2                        # 스위치 0/1을 서보 명령 1/2로 변환한다.

        try:
            set_servo(servo_command)                                         # 외부에서 사용할 단일 함수로 서보를 제어한다.
        except (RuntimeError, ValueError) as error:
            self.get_logger().error(f"서보 제어 실패: {error}")              # GPIO 및 입력 오류를 ROS 로그로 알린다.
            return

        angle = 90 if servo_command == 1 else 180                            # 로그에 표시할 실제 목표 각도를 정한다.
        self.get_logger().info(f"스위치={switch_value}, 서보={angle}도")      # 적용한 스위치와 각도를 기록한다.


# /* ROS 2 시험 노드를 실행한다. */
def main(args=None):
    rclpy.init(args=args)                                                    # ROS 2 통신을 초기화한다.
    node = ServoSwitchTestNode()                                             # 서보 시험 노드를 생성한다.

    try:
        rclpy.spin(node)                                                     # 종료 요청까지 스위치 메시지를 처리한다.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()                                                  # 노드가 점유한 ROS 자원을 해제한다.
        rclpy.shutdown()                                                     # ROS 2 통신을 종료한다.


if __name__ == "__main__":
    main()                                                                   # 파일을 직접 실행할 때 시험 노드를 시작한다.
