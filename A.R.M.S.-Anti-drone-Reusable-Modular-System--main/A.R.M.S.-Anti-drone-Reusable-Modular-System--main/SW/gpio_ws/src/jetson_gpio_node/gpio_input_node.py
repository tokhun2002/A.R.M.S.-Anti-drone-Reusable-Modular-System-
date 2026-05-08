import Jetson.GPIO as GPIO
import time

PIN = 29

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.IN)

print("GPIO input test start")

try:
    while True:
        state = GPIO.input(PIN)

        if state:
            print("1")
        else:
            print("0")

        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()