import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from simple_pid import PID

pins = {
    "red_left" : 33,
    "green_right" : 32,
    "forward" : 31,
    "backward" : 29
}

class Driver(Node):
    def __init__(self):
        # ROS2 node setup
        super().__init__('driver_node')
        self.create_subscription(AckermannDrive, 'drive', self.callback, 10)
        self.create_subscription(AckermannDrive, 'feedback', self.feedback_callback, 1)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

        self.pin_setup()

        self.feedback_ang = 45 # actual current angle
        self.ang = 45 # desired angle (starting angle)
        self.thresh = 4 # allowed angle difference
        self.e = 0 # current error (angle difference)
        self.speed = 0
        self.last_received = self.get_clock().now().nanoseconds / 1e9
        self.u = 0 # output from PID

        self.pid = PID(
            Kp=10, Ki=5, Kd=20, 
            setpoint=0, 
            output_limits=(-98.5, 98.5) # motors do not respond to values in range ~ 99-100
        )

    def pin_setup(self):
        try:
            GPIO.setmode(GPIO.BOARD)
            for _, pin in pins.items():
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        except Exception as e:
            print("GPIO setup failed:", e)
            GPIO.cleanup()
            raise e
        
        self.left = GPIO.PWM(pins["red_left"], 100)
        self.right = GPIO.PWM(pins["green_right"], 100)
        self.left.start(0)
        self.right.start(0)

    # Ackermann Drive callback
    def callback(self, msg: AckermannDrive):
        self.ang = int(msg.steering_angle) # keyboard input angle
        self.speed = int(msg.speed)
        self.last_received = self.get_clock().now().nanoseconds / 1e9

    # MRP sensor callback
    def feedback_callback(self, msg: AckermannDrive):
        self.feedback_ang = int(msg.steering_angle) # sensor angle
        self.e = self.ang - self.feedback_ang

        if abs(self.e) <= self.thresh:
            self.u = 0 # to prevent accumulation (has an effect if Ki > 0)
        else:
            self.u = self.pid(self.e)

    def update(self):
        # Reset conditions
        if self.get_clock().now().nanoseconds / 1e9 - self.last_received > 2:
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
            
            self.get_logger().info("Turning Left.")

        elif self.e < -self.thresh:
            self.left.ChangeDutyCycle(0)

            if abs(self.u) > 60:
                self.right.ChangeDutyCycle(abs(self.u))
            else:
                self.right.ChangeDutyCycle(0)
            
            self.get_logger().info("Turning Right.")

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

def main():
    rclpy.init()
    driver_node = Driver()
    try:
        rclpy.spin(driver_node)

    except KeyboardInterrupt:
        driver_node.get_logger().info("Shutting down driver node.")
    finally:
        driver_node.left.ChangeDutyCycle(0)
        driver_node.right.ChangeDutyCycle(0)
        driver_node.left.stop()
        driver_node.right.stop()
        GPIO.output(pins["forward"], GPIO.LOW)
        GPIO.output(pins["backward"], GPIO.LOW)
        GPIO.cleanup()
        driver_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
