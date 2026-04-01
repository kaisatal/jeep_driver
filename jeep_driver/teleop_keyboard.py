#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import threading
import sys
from select import select
import termios
import tty

msg = """
Reading from the keyboard and Publishing to /keyboard
---------------------------
Moving around:
      w
a           d
      s

CTRL-C to quit
"""

moveBindings = {'a', 'd', 'w', 's', ' '} # ' ' is for switching between driving controls


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        
        self.publisher = self.create_publisher(AckermannDrive, 'keyboard', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd) # 10 Hz

        # Parameters
        self.ang = 45
        self.speed = 0.0
        # Magnet value (angle) range depends on how the magnet is situated, so the range might change
        self.min_steering_deg = -20
        self.max_steering_deg = 75

        # To restore terminal settings afterwards
        self.settings = termios.tcgetattr(sys.stdin)

        # Keyboard thread
        self.running = True
        self.input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.input_thread.start()

        print(msg) # Info message

    def publish_cmd(self):
        drive_msg = AckermannDrive()
        drive_msg.steering_angle = float(self.ang)
        drive_msg.speed = float(self.speed)
        self.publisher.publish(drive_msg)

        self.speed = 0.0 # Reset
    
    def keyboard_loop(self):
        try:
            while self.running:
                key = getKey(self.settings, 0.1)

                if key in moveBindings:
                    if key == 'd' and self.ang > self.min_steering_deg:
                        self.ang -= 5
                    elif key == 'a' and self.ang < self.max_steering_deg:
                        self.ang += 5
                    elif key == 'w':
                        self.speed = 1.0
                    elif key == 's':
                        self.speed = -1.0

                    self.get_logger().info("Keyboard angle:", self.ang)

                elif key == '\x03':  # Ctrl+C
                    break

        except Exception as e:
            print(e)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Keyboard node.")
    finally:
        node.running = False
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()

if __name__=="__main__":
    main()
