# import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO
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
            # left.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
            time.sleep(0.01)
        time.sleep(0.5)

        right.ChangeDutyCycle(0)
        
        for duty in range(0,101,1):
            # right.ChangeDutyCycle(duty)
            left.ChangeDutyCycle(duty)
            time.sleep(0.01)
        time.sleep(0.5)

        left.ChangeDutyCycle(0)

def NewTest():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD) 
    GPIO.setup(pins["red_left"], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(pins["green_right"], GPIO.OUT, initial=GPIO.LOW)
    # go = pins["white_throttle"]
    right = pins["green_right"]
    left = pins["red_left"]
    # GPIO.output(go, GPIO.HIGH)
    GPIO.output(right, GPIO.LOW)
    GPIO.output(left, GPIO.HIGH)
    time.sleep(0.5)
    # GPIO.output(go, GPIO.LOW)
    # for _, pin in pins.items():
    #     GPIO.output(pin, GPIO.HIGH)
    

    # left.start(0)				#start PWM of required Duty Cycle 
    # right.start(0)
    # while True:
    #     # left.ChangeDutyCycle(100)
    #     right.ChangeDutyCycle(10)
    #     time.sleep(0.01)
    # left.ChangeDutyCycle(duty)

def init():
    global right, left

    # GPIO.setwarnings(False)
    # GPIO.setmode(GPIO.BOARD) 

    # right = pins["green_right"]
    # left = pins["red_left"]

    GPIO.setup(pins["red_left"], GPIO.OUT, initial=GPIO.LOW)

    GPIO.setup(pins["green_right"], GPIO.OUT, initial=GPIO.LOW)

    right = GPIO.PWM(pins["green_right"], 100)
    left = GPIO.PWM(pins["red_left"], 100)

    right.start(0)
    left.start(0)

    time.sleep(1)
    right.ChangeDutyCycle(100)
    time.sleep(1)
    right.ChangeDutyCycle(0)
    left.ChangeDutyCycle(100)
    time.sleep(1)
    left.ChangeDutyCycle(0)

    # GPIO.setup(pins["green_right"], GPIO.OUT)
    # GPIO.output(pins["green_right"], GPIO.LOW)

    # GPIO.setup(pins["red_left"], GPIO.OUT)
    # GPIO.output(pins["red_left"], GPIO.HIGH)

    # left.start(1)
    # time.sleep(1)
    # left.ChangeDutyCycle(100)
    # time.sleep(1)


def left_right():
    global right, left
    right = GPIO.PWM(pins["green_right"], 100)
    left = GPIO.PWM(pins["red_left"], 100)
    right.start(0)
    left.start(0)
    while True:
        turn = input("1 - left / 2 - right: ")
        if int(turn)<0:
            print("Turnng left")
            # GPIO.output(pins["green_right"], GPIO.LOW)
            # GPIO.output(pins["red_left"], GPIO.HIGH)
            right.ChangeDutyCycle(0)
            left.ChangeDutyCycle(abs(int(turn)))
        else:
            print("Turnng right")
            # GPIO.output(pins["red_left"], GPIO.LOW)
            # GPIO.output(pins["green_right"], GPIO.HIGH)
            left.ChangeDutyCycle(0)
            right.ChangeDutyCycle(abs(int(turn)))
        time.sleep(0.5)
        # GPIO.output(pins["red_left"], GPIO.LOW)
        # GPIO.output(pins["green_right"], GPIO.LOW)
        left.ChangeDutyCycle(0)
        right.ChangeDutyCycle(0)






def main():
    # global right, left
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD) 
    for _, pin in pins.items():
        print(f"Setting pin {_} to LOW")
        GPIO.setup(pin, GPIO.OUT)  # LED pin set as output
        GPIO.output(pin, GPIO.LOW)
    # GPIO.setup(pins["white_throttle"], GPIO.OUT)  # LED pin set as output
    # GPIO.output(pins["white_throttle"], GPIO.LOW)

    try:
        # init()
        # SimpleLEDTest()
        # PWMTest()
        # NewTest()
        left_right()

    except KeyboardInterrupt:
        print("\nStop")
        right.ChangeDutyCycle(0)
        left.ChangeDutyCycle(0)
        # for _, pin in pins.items():
        #     GPIO.output(pin, GPIO.LOW)
        right.stop()
        left.stop()
        # GPIO.cleanup()  # cleanup all GPIO

if __name__ == '__main__':
    main()