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

        self.feedback_ang = 0 # actual current angle
        self.thresh = 4 # allowed angle difference
        self.ang = 45 # desired angle (starting angle)
        self.speed = 0
        self.recive_time = rospy.get_time()
        self.u = 0 # output from PID

        self.n_left = 0 # +1 with left turn, -1 with no turn (still), set to 0 when n_right=30
        self.n_right = 0 # +1 with right turn, -1 with no turn (still), set to 0 when n_left=30
        self.is_oscillating = False # lower output value while True

        self.pid = PID(
            Kp=25, Ki=0, Kd=0, 
            setpoint=0, 
            output_limits=(-98.5, 100)
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
        self.ang = int(data.steering_angle)
        self.speed = int(data.speed)
        self.recive_time = rospy.get_time()
        self.pid.setpoint = self.ang

    # MRP sensor callback
    def feedback_callback(self, data):
        self.feedback_ang = int(data.steering_angle)

        if abs(self.ang - self.feedback_ang) <= self.thresh:
            self.u = 0 # to prevent accumulation
        else:
            self.u = self.pid(self.feedback_ang)

    def update(self):
        e = self.ang - self.feedback_ang

        # Reset conditions
        if rospy.get_time() - self.recive_time > 2:
            self.speed = 0

        # Current magnet value range: -17 to 73

        # Magnet value range (for angle) depends on how the magnet is situated, but the hot
        # glue that held the magnet has come loose, therefore the magnet can move and the
        # range might change

        # Turning
        print(f"dest: {self.ang}, curr: {self.feedback_ang}, output: {self.u}")
        if e > self.thresh:
            self.right.ChangeDutyCycle(0)

            if self.is_oscillating:
                self.left.ChangeDutyCycle(0)
                rospy.loginfo("Oscillation prevention (Turning Left).")
            else:
                self.left.ChangeDutyCycle(abs(self.u))
                rospy.loginfo("Turning Left.")

            self.n_left += 1
            if self.n_left >= 30:
                self.n_right = 0 # Not oscillating

        elif e < -self.thresh:
            self.left.ChangeDutyCycle(0)

            if self.is_oscillating:
                self.right.ChangeDutyCycle(0)
                rospy.loginfo("Oscillation prevention (Turning Right).")
            else:
                self.right.ChangeDutyCycle(abs(self.u)) # does not respond to values in range 99-100
                rospy.loginfo("Turning Right.")

            self.n_right += 1
            if self.n_right >= 30:
                self.n_left = 0 # Not oscillating

        else:
            self.right.ChangeDutyCycle(0)
            self.left.ChangeDutyCycle(0)

            if self.n_left > 0:
                self.n_left -= 1
            if self.n_right > 0:
                self.n_right -= 1

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
