#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32
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

moveBindings = {'a', 'd', 'w', 's'}

input_choice = {1: "/keyboard_drive", 2: "/cmd_drive"} # For informative logging


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
        
        self.keyboard_pub = self.create_publisher(AckermannDrive, 'keyboard_drive', 10)
        self.keyboard_timer = self.create_timer(0.1, self.publish_drive) # 10 Hz
        self.choice_pub = self.create_publisher(Int32, 'input_choice', 10)
        self.choice_timer = self.create_timer(0.5, self.publish_choice) # 2 Hz

        # Parameters
        self.ang = 45
        self.speed = 0.0

        # Magnet value (angle) range depends on how the magnet is situated, so this range might change
        self.min_steering_deg = -20
        self.max_steering_deg = 75

        self.choice_value = 1 # Input choice initial value

        # To restore terminal settings afterwards
        self.settings = termios.tcgetattr(sys.stdin)

        # Keyboard thread
        self.running = True
        self.input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.input_thread.start()

        print(msg) # Info message

    def publish_drive(self):
        drive_msg = AckermannDrive()
        drive_msg.steering_angle = float(self.ang)
        drive_msg.speed = float(self.speed)
        self.keyboard_pub.publish(drive_msg)
        # Reset the speed
        self.speed = 0.0
    
    def publish_choice(self):
        choice_msg = Int32()
        choice_msg.data = self.choice_value
        self.choice_pub.publish(choice_msg)
    
    def keyboard_loop(self):
        try:
            while self.running:
                key = getKey(self.settings, 0.1)

                if key in moveBindings:
                    if key == 'd' and self.ang > self.min_steering_deg:
                        self.ang -= 5
                        self.get_logger().info("Angle set to: ", self.ang)
                    elif key == 'a' and self.ang < self.max_steering_deg:
                        self.ang += 5
                        self.get_logger().info("Angle set to: ", self.ang)
                    elif key == 'w':
                        self.speed = 1.0
                        self.get_logger().info("Speed set to: 1")
                    elif key == 's':
                        self.speed = -1.0
                        self.get_logger().info("Speed set to: -1")
                
                elif key == ' ': # Separate topic to publish switch between driving controls
                    # Toggle between 1 and 2
                    if self.choice_value == 1:
                        self.choice_value = 2
                    else:
                        self.choice_value = 1
                    
                    # 1 - /keyboard, 2 - /cmd_drive
                    self.get_logger().info("Input choice set to: ", input_choice[self.choice_value])

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
        node.get_logger().info("Shutting down the Keyboard node.")
    finally:
        node.running = False
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()

if __name__=="__main__":
    main()
