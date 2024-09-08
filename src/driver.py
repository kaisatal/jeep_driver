import RPi.GPIO as GPIO
import rospy
from ackermann_msgs.msg import AckermannDrive
from simple_pid import PID

pins = {
    "red_left" : 33,
    "green_right" : 32,
    "forward" : 31,
    "backward" : 29
}

class Driver:
    def __init__(self):
        self.pin_setup()

        # ROS node setup
        rospy.init_node('driver_node')
        rospy.Subscriber("drive", AckermannDrive, self.callback)
        rospy.Subscriber("feedback", AckermannDrive, self.feedback_callback, buff_size=1)
        r = rospy.Rate(100)

        self.feedback_ang = 0
        self.thresh = 5
        self.ang = 0
        self.speed = 0
        self.recive_time = rospy.get_time()
        self.u = 0

        self.pid = PID(
            Kp=1.6, Ki=0.8, Kd=1, 
            setpoint=0, 
            output_limits=(-100, 100)
        )

        while True:
            self.update()
            r.sleep()
            if rospy.is_shutdown():
                rospy.loginfo("Stoping the driver.")
                self.right.ChangeDutyCycle(0)
                self.left.ChangeDutyCycle(0)
                self.right.stop()
                self.left.stop()
                GPIO.output(pins["forward"], GPIO.LOW)
                GPIO.output(pins["backward"], GPIO.LOW)
                break

    def pin_setup(self):
        GPIO.setmode(GPIO.BOARD) 
        for _, pin in pins.items():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self.right = GPIO.PWM(pins["green_right"], 100)
        self.left = GPIO.PWM(pins["red_left"], 100)
        self.left.start(0)
        self.right.start(0)

    # Ackermann Drive callback
    def callback(self, data):
        self.ang = int(data.steering_angle)
        self.speed = int(data.speed)
        self.recive_time = rospy.get_time()
        self.pid.setpoint = self.ang

    # MRP sensor callback
    def feedback_callback(self, data):
        self.feedback_ang = int(data.steering_angle)
        self.u = self.pid(self.feedback_ang)

    def update(self):
        e = self.ang - self.feedback_ang

        # Reset conditions
        if rospy.get_time() - self.recive_time > 2:
            self.speed = 0
        elif self.feedback_ang > 55 and e > 0:
            self.ang = self.feedback_ang
            e = 0
        elif self.feedback_ang < 15 and e < -0:
            self.ang = self.feedback_ang
            e = 0

        # Turning
        print(self.ang, self.feedback_ang, self.u)
        if e > self.thresh:
            self.right.ChangeDutyCycle(0)
            self.left.ChangeDutyCycle(abs(self.u))
            rospy.loginfo("Turning Left.")
        elif e < -self.thresh:
            self.left.ChangeDutyCycle(0)
            self.right.ChangeDutyCycle(abs(self.u))
            rospy.loginfo("Turning Right.")
        else:
            self.right.ChangeDutyCycle(0)
            self.left.ChangeDutyCycle(0)

        # Driving
        if self.speed > 0:
            GPIO.output(pins["forward"], GPIO.HIGH)
            GPIO.output(pins["backward"], GPIO.LOW)
        elif self.speed < 0:
            GPIO.output(pins["forward"], GPIO.HIGH)
            GPIO.output(pins["backward"], GPIO.HIGH)
        else:
            GPIO.output(pins["forward"], GPIO.LOW)
            GPIO.output(pins["backward"], GPIO.LOW)
    
if __name__ == '__main__':
    Driver()