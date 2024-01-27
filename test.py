import Jetson.GPIO as GPIO
import time

# Pin Definitons:
pins = {
    "red_left" : 33,
    "green_right" : 32,
    # "white_throttle" : 31
}

test = 0

def Test1():
    global test, right, left

    test = 1
    right = GPIO.PWM(pins["green_right"], 50)
    left = GPIO.PWM(pins["red_left"], 50)


    left.start(0)
    right.start(0)
    while True:
        rotate = input("left right: ")
        rotate_l = int(rotate.split()[0])
        rotate_r = int(rotate.split()[1])

        left.ChangeDutyCycle(rotate_l)
        right.ChangeDutyCycle(rotate_r)

def Test2():
    global test

    test = 2
    while True:
        rotate = input("left right: ")

        if rotate == 'l':
            GPIO.output(pins['green_right'], GPIO.LOW)
            GPIO.output(pins['red_left'], GPIO.HIGH)
        elif rotate == 'r':
            GPIO.output(pins['red_left'], GPIO.LOW)
            GPIO.output(pins['green_right'], GPIO.HIGH)

def main():
    global left, right 

    GPIO.setmode(GPIO.BOARD) 

    for _, pin in pins.items():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    try:
        Test1()
        # Test2()

    except KeyboardInterrupt:
        print("\nStop")
        if test == 1:
            for _, pin in pins.items():
                GPIO.output(pin, GPIO.LOW)
        elif test == 2:
            right.ChangeDutyCycle(0)
            left.ChangeDutyCycle(0)
            right.stop()
            left.stop()
        GPIO.cleanup() 

if __name__ == '__main__':
    main()