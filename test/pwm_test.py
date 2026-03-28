import Jetson.GPIO as GPIO
import time

pins = {
    "red_left": 33,
    "green_right": 32,
    "forward": 31,
    "backward": 29
}

GPIO.setmode(GPIO.BOARD)

steering_pins = [pins["red_left"], pins["green_right"]]
driving_pins = [pins["forward"], pins["backward"]]

for pin in driving_pins:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

for pin in steering_pins:
    GPIO.setup(pin, GPIO.OUT)

pwms = [GPIO.PWM(p, 1000) for p in steering_pins]

for pwm in pwms:
    pwm.start(0)

try:
    while True:
        print("HIGH")
        for pin in driving_pins:
            GPIO.output(pin, GPIO.HIGH)

        for dc in range(0, 101, 2):
            for pwm in pwms:
                pwm.ChangeDutyCycle(dc)
            time.sleep(0.2)

        print("LOW")
        for pin in driving_pins:
            GPIO.output(pin, GPIO.HIGH)
        for dc in range(100, -1, -2):
            for pwm in pwms:
                pwm.ChangeDutyCycle(dc)
            time.sleep(0.2)
finally:
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()
