import Jetson.GPIO as GPIO
import time

PIN = 29

GPIO.setmode(GPIO.BOARD)

# 입력 모드
GPIO.setup(PIN, GPIO.IN)

print("GPIO INPUT TEST")

try:
    while True:
        state = GPIO.input(PIN)

        if state:
            print("HIGH (3.3V 입력 감지)")
        else:
            print("LOW")

        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()
