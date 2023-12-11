import RPi.GPIO as GPIO
import time

# Pin Definitons:
pins = {
    "red_left" : 33,
    "green_right" : 32,
    "white_throttle" : 31
}

def SimpleLEDTest():
    index = 0
    while True:
        index += 1
        if index%2 == 0:
            for _, pin in pins.items():
                GPIO.output(pin, GPIO.HIGH)
        else:
            for _, pin in pins.items():
                GPIO.output(pin, GPIO.LOW)
        time.sleep(1)

def PWMTest():
    right = GPIO.PWM(pins["green_right"], 1000)
    left = GPIO.PWM(pins["red_left"], 1000)

    left.start(0)				#start PWM of required Duty Cycle 
    right.start(0)
    while True:
        for duty in range(0,101,1):
            right.ChangeDutyCycle(duty)
            left.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
            time.sleep(0.01)
        time.sleep(0.5)
        
        for duty in range(100,-1,-1):
            right.ChangeDutyCycle(duty)
            left.ChangeDutyCycle(duty)
            time.sleep(0.01)
        time.sleep(0.5)


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD) 
    for _, pin in pins.items():
        GPIO.setup(pin, GPIO.OUT)  # LED pin set as output
        GPIO.output(pin, GPIO.LOW)

    try:
        SimpleLEDTest()
        # PWMTest()

    except KeyboardInterrupt:
        print("\nStop")
        for _, pin in pins.items():
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()  # cleanup all GPIO

if __name__ == '__main__':
    main()