import RPi.GPIO as GPIO
import rospy
from ackermann_msgs.msg import AckermannDrive

pins = {
    "red_left" : 33,
    "green_right" : 32,
    "white_throttle" : 31
}

def callback(data):
    global ang, speed, recive_time

    ang = int(data.steering_angle)
    speed = int(data.speed)

    rospy.loginfo(f"Ang:{ang}")

    recive_time = rospy.get_time()

def main():
    global ang, speed, recive_time

    GPIO.setmode(GPIO.BOARD) 

    for _, pin in pins.items():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    
    right = GPIO.PWM(pins["green_right"], 100)
    left = GPIO.PWM(pins["red_left"], 100)

    left.start(0)
    right.start(0)

    rospy.init_node('driver_node')
    rospy.Subscriber("drive", AckermannDrive, callback)

    ang = 0
    speed = 0
    recive_time = rospy.get_time()


    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        if current_time - recive_time > 0.3:
            ang = 0
            speed = 0

        if ang>0:
            left.ChangeDutyCycle(0)
            right.ChangeDutyCycle(abs(ang))
        elif ang<0:
            right.ChangeDutyCycle(0)
            left.ChangeDutyCycle(abs(ang))
        else:
            right.ChangeDutyCycle(0)
            left.ChangeDutyCycle(0)

        if speed > 0:
            GPIO.output(pins["white_throttle"], GPIO.HIGH)
        else:
            GPIO.output(pins["white_throttle"], GPIO.LOW)
    
    # rospy.spin()

if __name__ == '__main__':
    main()