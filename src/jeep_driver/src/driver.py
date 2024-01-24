import RPi.GPIO as GPIO
import rospy
from ackermann_msgs.msg import AckermannDrive

pins = {
    "red_left" : 33,
    "green_right" : 32,
    "white_throttle" : 31
}

def callback(data):
    ang = int(data.steering_angle)
    speed = int(data.speed)
    if ang>0:
        left.ChangeDutyCycle(0)
        right.ChangeDutyCycle(ang)
    else:
        right.ChangeDutyCycle(0)
        left.ChangeDutyCycle(-ang)
    # rospy.loginfo(f'{ang}, {speed}')

    if speed == 1:
        GPIO.output(pins["white_throttle"], GPIO.HIGH)
    else:
        GPIO.output(pins["white_throttle"], GPIO.LOW)

    
def main():
    global left, right

    GPIO.setmode(GPIO.BOARD) 

    for _, pin in pins.items():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    
    right = GPIO.PWM(pins["green_right"], 1000)
    left = GPIO.PWM(pins["red_left"], 1000)


    left.start(0)
    right.start(0)

    rospy.init_node('driver_node', anonymous=True)
    rospy.Subscriber("drive", AckermannDrive, callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()