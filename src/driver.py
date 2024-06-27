import RPi.GPIO as GPIO
import rospy
from ackermann_msgs.msg import AckermannDrive


pins = {
    "red_left" : 33,
    "green_right" : 32,
    "forward" : 31,
    "backward" : 29
}
feedback_ang = 0
thresh = 10

# Ackermann Drive callback
def callback(data):
    global ang, speed, recive_time
    ang = int(data.steering_angle)
    speed = int(data.speed)
    recive_time = rospy.get_time()

# MRP sensor callback
def feedback_callback(data):
    global feedback_ang
    feedback_ang = int(data.steering_angle)

def main():
    global ang, speed, recive_time, feedback_ang

    # Pin setup
    GPIO.setmode(GPIO.BOARD) 
    for _, pin in pins.items():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    right = GPIO.PWM(pins["green_right"], 100)
    left = GPIO.PWM(pins["red_left"], 100)
    left.start(0)
    right.start(0)

    # ROS node setup
    rospy.init_node('driver_node')
    rospy.Subscriber("drive", AckermannDrive, callback)
    rospy.Subscriber("feedback", AckermannDrive, feedback_callback, buff_size=1)
    r = rospy.Rate(100)

    # Initialisation
    ang = 0
    speed = 0
    recive_time = rospy.get_time()
    t0 = rospy.get_time()
    I = 0
    e0 = ang - feedback_ang
    Kp, Ki, Kd = 1.3, 0.3, 0.02

    while not rospy.is_shutdown():
        current_time = rospy.get_time()

        # PID controller
        dt = rospy.get_time()-t0
        t0 += dt
        e = ang - feedback_ang
        D = (e0-e)/dt
        if e>thresh or e<-thresh:
            I += e*dt
        u = Kp*e + Ki*I + Kd*D
        if abs(u) > 100:
            u = 100
            # I = 0
        # print(e, I, D)
        # print(u)
        # print("0:",  feedback_ang, ang, e)

        # Reset conditions
        if current_time - recive_time > 2:
            speed = 0
        elif feedback_ang > 55 and e > 0:
            ang = feedback_ang
            e = 0
        elif feedback_ang < 15 and e < -0:
            ang = feedback_ang
            e = 0

        # Turning
        if e > thresh:
            right.ChangeDutyCycle(0)
            left.ChangeDutyCycle(abs(u))
            rospy.loginfo("Turning Left.")
        elif e < -thresh:
            left.ChangeDutyCycle(0)
            right.ChangeDutyCycle(abs(u))
            rospy.loginfo("Turning Right.")
        else:
            right.ChangeDutyCycle(0)
            left.ChangeDutyCycle(0)

        # Driving
        if speed > 0:
            GPIO.output(pins["forward"], GPIO.HIGH)
            GPIO.output(pins["backward"], GPIO.LOW)
        elif speed < 0:
            GPIO.output(pins["forward"], GPIO.HIGH)
            GPIO.output(pins["backward"], GPIO.HIGH)
        else:
            GPIO.output(pins["forward"], GPIO.LOW)
            GPIO.output(pins["backward"], GPIO.LOW)

        r.sleep()
        if rospy.is_shutdown():
            rospy.loginfo("Stoping the driver.")
            right.ChangeDutyCycle(0)
            left.ChangeDutyCycle(0)
            right.stop()
            left.stop()
            GPIO.output(pins["forward"], GPIO.LOW)
            GPIO.output(pins["backward"], GPIO.LOW)
            break
    
if __name__ == '__main__':
    main()