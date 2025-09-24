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

        self.feedback_ang = 45 # actual current angle
        self.ang = 45 # desired angle (starting angle)
        self.thresh = 4 # allowed angle difference
        self.e = 0 # current error (angle difference)
        self.speed = 0
        self.receive_time = rospy.get_time()
        self.u = 0 # output from PID

        self.pid = PID(
            Kp=10, Ki=5, Kd=20, 
            setpoint=0, 
            output_limits=(-98.5, 98.5) # motors do not respond to values in range ~ 99-100
        )

        while True:
            self.update()
            r.sleep()
            if rospy.is_shutdown():
                rospy.loginfo("Stopping the driver.")
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
        self.ang = int(data.steering_angle) # keyboard input angle
        self.speed = int(data.speed)
        self.receive_time = rospy.get_time()

    # MRP sensor callback
    def feedback_callback(self, data):
        self.feedback_ang = int(data.steering_angle) # sensor angle
        self.e = self.ang - self.feedback_ang

        if abs(self.e) <= self.thresh:
            self.u = 0 # to prevent accumulation (has an effect if Ki > 0)
        else:
            self.u = self.pid(self.e)

    def update(self):
        # Reset conditions
        if rospy.get_time() - self.receive_time > 2:
            self.speed = 0

        # Current magnet value range: -17 to 73

        # Magnet value range (for angle) depends on how the magnet is situated, but the hot
        # glue that held the magnet has come loose, therefore the magnet can move and the
        # range might change

        # Turning
        print(f"dest: {self.ang}, curr: {self.feedback_ang}, output: {self.u}")

        if self.e > self.thresh:
            self.right.ChangeDutyCycle(0)

            if abs(self.u) > 60:
                self.left.ChangeDutyCycle(abs(self.u))
            else:
                self.left.ChangeDutyCycle(0)
            
            rospy.loginfo("Turning Left.")

        elif self.e < -self.thresh and abs(self.u) > 60:
            self.left.ChangeDutyCycle(0)

            if abs(self.u) > 60:
                self.right.ChangeDutyCycle(abs(self.u))
            else:
                self.right.ChangeDutyCycle(0)
            
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
