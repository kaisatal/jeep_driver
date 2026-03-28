import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from simple_pid import PID
import Jetson.GPIO as GPIO
import threading
import time
import os

pins = {
    "red_left" : 33,
    "green_right" : 32,
    "forward" : 31,
    "backward" : 29
}

# Pin 33 won't work with GPIO.PWM, using sysfs commands directly
class SysfsPWM:
    def __init__(self, chip, channel):
        self.base = f"/sys/class/pwm/pwmchip{chip}"
        self.channel = channel
        self.path = f"{self.base}/pwm{channel}"

        # force reset
        if os.path.exists(self.path):
            with open(f"{self.base}/unexport", "w") as f:
                f.write(str(channel))
            time.sleep(0.1)

        with open(f"{self.base}/export", "w") as f:
            f.write(str(channel))
        time.sleep(0.1) # allow sysfs to create files

        self.period = 20_000_000  # 20 ms period (50 Hz)

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

# Needed to not block ROS callbacks
class PWMWorker:
    def __init__(self):
        self.left = SysfsPWM(0, 2)
        self.right = SysfsPWM(0, 0)

        self.left_duty = 0
        self.right_duty = 0

        # To send duty change only after actual change
        self.prev_l = 0
        self.prev_r = 0

        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def set(self, left, right):
        with self.lock:
            self.left_duty = left
            self.right_duty = right

    def loop(self):
        while self.running:
            if self.left_duty != self.prev_l or self.right_duty != self.prev_r:
                with self.lock:
                    l = self.left_duty
                    r = self.right_duty

                self.left.duty(l)
                self.right.duty(r)

                self.prev_l = l
                self.prev_r = r

            time.sleep(0.02)  # 50 Hz output rate

    def stop(self):
        self.running = False
        self.thread.join()
        self.left.duty(0)
        self.right.duty(0)
        self.left.stop()
        self.right.stop()


class DriverNode(Node):
    def __init__(self):
        # ROS2 node setup
        super().__init__('driver_node')

        self.gpio_ready = False # To prevent timer race condition
        self.pin_setup()

        self.create_subscription(AckermannDrive, 'drive', self.callback, 10)
        self.create_subscription(AckermannDrive, 'feedback', self.feedback_callback, 1)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

        self.feedback_ang = 45 # actual current angle
        self.keyboard_ang = 45 # desired angle (starting angle)
        self.thresh = 4 # allowed angle difference
        self.e = 0 # current error (keyboard_ang - feedback_ang)
        self.speed = 0
        self.last_received = self.get_clock().now().nanoseconds / 1e9
        self.u = 0 # output from PID

        self.pid = PID(
            Kp = 10, Ki = 5, Kd = 20, 
            setpoint = 0,
            output_limits = (-98.5, 98.5) # motors do not respond to values in range ~ 99-100
        )

        self.pwm = PWMWorker() # sysfs commands

    def pin_setup(self):
        GPIO.setmode(GPIO.BOARD)

        for name, pin in pins.items():
            if name in ["red_left", "green_right"]:
                continue  # skip PWM pins
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

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

        print(f"dest: {self.keyboard_ang}, curr: {self.feedback_ang}, output: {self.u}")

        # Current magnet sensor value range: -15 to 75

        # Magnet value range (for angle) depends on how the magnet is situated, but the hot
        # glue that held the magnet has come loose, therefore the magnet can move and the
        # range might change

        # Turning
        if self.e > self.thresh: # To prevent oscillation
            if abs(self.u) > 60: # To prevent low values straining the motor
                self.pwm.set(abs(self.u), 0)
                print("Turning left")
            else:
                self.pwm.set(0, 0)
            #self.get_logger().info("Turning Left.")

        elif self.e < -self.thresh:
            if abs(self.u) > 60:
                self.pwm.set(0, abs(self.u))
                print("Turning right")
            else:
                self.pwm.set(0, 0)
            #self.get_logger().info("Turning Right.")

        else:
            self.pwm.set(0, 0)

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
    node = DriverNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down driver node.")

    finally:
        node.timer.cancel()
        node.pwm.stop()
        time.sleep(1)
        node.destroy_node()

        GPIO.output(pins["forward"], GPIO.LOW)
        GPIO.output(pins["backward"], GPIO.LOW)
        # No cleanup because it sets the driving pins high (hardware pull-up)
        #GPIO.cleanup()
        while True:
            time.sleep(1)
            # Second Ctrl+C only after turning off the car!

if __name__ == '__main__':
    main()
