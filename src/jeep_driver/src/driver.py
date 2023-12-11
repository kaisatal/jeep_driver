import RPi.GPIO as GPIO
import time

import rospy
from ackermann_msgs.msg import AckermannDrive

pins = {
    "red_left" : 33,
    "green_right" : 32,
    "white_throttle" : 31
}

def callback(data):
    ang = int(data.steering_angle)
    if ang>0:
        right.ChangeDutyCycle(ang)
        left.ChangeDutyCycle(0)
    else:
        right.ChangeDutyCycle(0)
        left.ChangeDutyCycle(-ang)
    rospy.loginfo(ang)
    
def main():
    global left, right
    GPIO.setmode(GPIO.BOARD) 
    for _, pin in pins.items():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    right = GPIO.PWM(pins["green_right"], 1000)
    left = GPIO.PWM(pins["red_left"], 1000)

    left.start(0)				#start PWM of required Duty Cycle 
    right.start(0)

    rospy.init_node('driver_node', anonymous=True)
    rospy.Subscriber("drive", AckermannDrive, callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()