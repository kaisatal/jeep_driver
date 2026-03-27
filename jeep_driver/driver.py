import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from simple_pid import PID
import os

pins = {
    "red_left" : 33,
    "green_right" : 32,
    "forward" : 31,
    "backward" : 29
}

# Replacing GPIO.PWM with sysfs because of initialization issues
class SysfsPWM:
    def __init__(self, chip, channel):
        self.base = f"/sys/class/pwm/pwmchip{chip}"
        self.channel = channel
        self.path = f"{self.base}/pwm{channel}"

        if not os.path.exists(self.path):
            with open(f"{self.base}/export", "w") as f:
                f.write(str(channel))

        self.period = 20000000  # 20 ms

        with open(f"{self.path}/period", "w") as f:
            f.write(str(self.period))

        with open(f"{self.path}/enable", "w") as f:
            f.write("1")

    def duty(self, percent):
        percent = max(0, min(100, percent))
        duty = int(self.period * percent / 100)
        with open(f"{self.path}/duty_cycle", "w") as f:
            f.write(str(duty))

    def stop(self):
        with open(f"{self.path}/enable", "w") as f:
            f.write("0")


class DriverNode(Node):
    def __init__(self):
        # ROS2 node setup
        super().__init__('driver_node')

        self.pin_setup()

        self.create_subscription(AckermannDrive, 'drive', self.callback, 10)
        self.create_subscription(AckermannDrive, 'feedback', self.feedback_callback, 1)
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.feedback_ang = 45 # actual current angle
        self.keyboard_ang = 45 # desired angle (starting angle)
        self.thresh = 4 # allowed angle difference
        self.e = 0 # current error (keyboard_ang - feedback_ang)
        self.speed = 0
        self.last_received = self.get_clock().now().nanoseconds / 1e9
        self.u = 0 # output from PID

        self.gpio_ready = False # To prevent timer race condition

        self.pid = PID(
            Kp = 10, Ki = 5, Kd = 20, 
            setpoint = 0,
            output_limits = (-98.5, 98.5) # motors do not respond to values in range ~ 99-100
        )

    def pin_setup(self):
        GPIO.setmode(GPIO.BOARD)

        for _, pin in pins.items():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        # Replacing GPIO.PWM
        self.left = SysfsPWM(0, 2) # pin 33, left
        self.right = SysfsPWM(0, 0) # pin 32, right

        self.gpio_ready = True

    # Ackermann Drive callback
    def callback(self, msg: AckermannDrive):
        self.keyboard_ang = int(msg.steering_angle) # keyboard input angle
        self.speed = int(msg.speed)
        self.last_received = self.get_clock().now().nanoseconds / 1e9

    # MRP sensor callback
    def feedback_callback(self, msg: AckermannDrive):
        self.feedback_ang = int(msg.steering_angle) # sensor angle
        self.e = self.keyboard_ang - self.feedback_ang
        if abs(self.e) <= self.thresh:
            self.u = 0 # to prevent accumulation (has an effect if Ki > 0)
        else:
            self.u = self.pid(self.e)

    def update(self):
        # Reset conditions
        if self.get_clock().now().nanoseconds / 1e9 - self.last_received > 2:
            self.speed = 0

        if not self.gpio_ready:
            return

        # Current magnet sensor value range: -15 to 75

        # Magnet value range (for angle) depends on how the magnet is situated, but the hot
        # glue that held the magnet has come loose, therefore the magnet can move and the
        # range might change

        # Turning
        print(f"dest: {self.keyboard_ang}, curr: {self.feedback_ang}, output: {self.u}")

        if self.e > self.thresh:
            self.right.duty(0)

            if abs(self.u) > 60:
                self.left.duty(abs(self.u))
            else:
                self.left.duty(0)
            #self.get_logger().info("Turning Left.")

        elif self.e < -self.thresh:
            self.left.duty(0)

            if abs(self.u) > 60:
                self.right.duty(abs(self.u))
            else:
                self.right.duty(0)
            #self.get_logger().info("Turning Right.")

        else:
            self.right.duty(0)
            self.left.duty(0)

        # Driving
        if self.speed > 0:
            GPIO.output(pins["forward"], GPIO.HIGH)
            GPIO.output(pins["backward"], GPIO.LOW)
        elif self.speed < 0:
            GPIO.output(pins["forward"], GPIO.LOW)
            GPIO.output(pins["backward"], GPIO.HIGH)
        else:
            GPIO.output(pins["forward"], GPIO.LOW)
            GPIO.output(pins["backward"], GPIO.LOW)

def main():
    rclpy.init()
    driver_node = DriverNode()
    try:
        rclpy.spin(driver_node)

    except KeyboardInterrupt:
        driver_node.get_logger().info("Shutting down driver node.")
    finally:
        driver_node.timer.cancel()

        driver_node.left.duty(0)
        driver_node.right.duty(0)

        driver_node.left.stop()
        driver_node.right.stop()

        GPIO.output(pins["forward"], GPIO.LOW)
        GPIO.output(pins["backward"], GPIO.LOW)
        GPIO.cleanup()

        driver_node.destroy_node()

if __name__ == '__main__':
    main()
self.left.duty(abs(self.u))
