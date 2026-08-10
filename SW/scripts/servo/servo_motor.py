"""Jetson 40핀 헤더의 물리 15번 핀으로 MG996R 서보를 제어한다."""

import atexit                                                                 # 종료 시 PWM을 안전하게 해제한다.

import Jetson.GPIO as GPIO                                                   # Jetson GPIO와 하드웨어 PWM을 사용한다.

__all__ = ["set_servo"]                                                     # 외부에 단일 제어 함수만 공개한다.

SERVO_PIN = 15                                                               # BOARD 기준 물리 15번 핀(GPIO12)을 사용한다.
PWM_FREQUENCY_HZ = 50                                                        # 서보 표준 주기인 20 ms를 사용한다.
MIN_PULSE_MS = 0.5                                                           # MG996R의 0도 기준 펄스 폭을 지정한다.
MAX_PULSE_MS = 2.5                                                           # MG996R의 180도 기준 펄스 폭을 지정한다.

_pwm = None                                                                  # 최초 호출 때 생성할 PWM 객체를 보관한다.


# /* PWM 자원을 초기화한다. */
def _initialize_pwm():
    global _pwm                                                              # 모듈 전체에서 같은 PWM 객체를 재사용한다.

    if _pwm is not None:                                                     # 이미 초기화했으면 기존 객체를 사용한다.
        return _pwm

    GPIO.setmode(GPIO.BOARD)                                                 # 핀 번호를 물리 헤더 번호로 해석한다.
    GPIO.setup(SERVO_PIN, GPIO.OUT)                                          # PWM 신호 핀을 출력으로 설정한다.
    _pwm = GPIO.PWM(SERVO_PIN, PWM_FREQUENCY_HZ)                             # 50 Hz 하드웨어 PWM 객체를 만든다.
    _pwm.start(0.0)                                                          # 초기 펄스 없이 PWM 출력을 시작한다.
    return _pwm


# /* 각도를 PWM 듀티비로 변환한다. */
def _angle_to_duty_cycle(angle):
    pulse_ms = MIN_PULSE_MS + (MAX_PULSE_MS - MIN_PULSE_MS) * angle / 180.0  # 각도를 0.5~2.5 ms로 변환한다.
    return pulse_ms / (1000.0 / PWM_FREQUENCY_HZ) * 100.0                    # 펄스 폭을 듀티비 백분율로 변환한다.


# /* 명령 1은 90도, 명령 2는 180도로 서보를 이동한다. */
def set_servo(command):
    if command not in (1, 2):                                                # 허용하지 않은 명령을 즉시 거부한다.
        raise ValueError("서보 명령은 1(90도) 또는 2(180도)만 가능하다.")

    angle = 90.0 if command == 1 else 180.0                                  # 외부 명령을 목표 각도로 변환한다.
    duty_cycle = _angle_to_duty_cycle(angle)                                 # 목표 각도에 맞는 듀티비를 계산한다.
    pwm = _initialize_pwm()                                                   # 최초 호출 시 GPIO와 PWM을 준비한다.
    pwm.ChangeDutyCycle(duty_cycle)                                           # 서보를 목표 각도로 이동하고 유지한다.


# /* 프로그램 종료 시 PWM과 GPIO를 정리한다. */
def _cleanup():
    global _pwm                                                              # 현재 PWM 객체를 해제한다.

    if _pwm is None:                                                         # 초기화하지 않았다면 정리하지 않는다.
        return

    _pwm.stop()                                                               # PWM 신호 출력을 중지한다.
    GPIO.cleanup(SERVO_PIN)                                                   # 사용한 물리 핀만 해제한다.
    _pwm = None                                                               # 다음 실행에서 새 객체를 만들도록 초기화한다.


atexit.register(_cleanup)                                                     # 정상 종료와 예외 종료에서 자원을 정리한다.
