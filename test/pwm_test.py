import Jetson.GPIO as GPIO
import time

pins = {
    "red_left": 33,
    "green_right": 32
}

GPIO.setmode(GPIO.BOARD)

steering_pins = [pins["red_left"], pins["green_right"]]

for pin in steering_pins:
    GPIO.setup(pin, GPIO.OUT)

pwms = [GPIO.PWM(p, 1000) for p in steering_pins]

for pwm in pwms:
    pwm.start(0)

try:
    while True:
        print("Rising")
        for dc in range(0, 101, 2):
            for pwm in pwms:
                pwm.ChangeDutyCycle(dc)
            time.sleep(0.2)

        print("Falling")
        for dc in range(100, -1, -2):
            for pwm in pwms:
                pwm.ChangeDutyCycle(dc)
            time.sleep(0.2)
finally:
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()
