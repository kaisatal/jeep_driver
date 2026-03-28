import Jetson.GPIO as GPIO
import time

pins = {
    "forward": 31,
    "backward": 29
}

GPIO.setmode(GPIO.BOARD)

fwd = pins["forward"]
bwd = pins["backward"]

GPIO.setup(fwd, GPIO.OUT)
GPIO.setup(bwd, GPIO.OUT)

try:
    while True:
        print("Forward (pin 31)")
        GPIO.output(fwd, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(fwd, GPIO.LOW)

        print("Backward (pin 29)")
        GPIO.output(bwd, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(bwd, GPIO.LOW)

except KeyboardInterrupt:
    pass

finally:
    # After cleanup() there is hardware pull-up
    # To keep at 0 V they need to be kept LOW with no cleanup
    GPIO.output(fwd, GPIO.LOW)
    GPIO.output(bwd, GPIO.LOW)

    while True:
        time.sleep(1)
