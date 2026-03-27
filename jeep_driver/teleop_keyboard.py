#!/usr/bin/env python
import threading
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import sys
from select import select
import termios
import tty

msg = """
Reading from the keyboard and Publishing to AckermannDrive!
---------------------------
Moving around:
      w
a           d
      s

CTRL-C to quit
"""

moveBindings = {
        'a': -1,
        'd': 1,
        'w': 0,
        's': 0
    }

class PublishThread(threading.Thread):
    def __init__(self, node, rate, ang):
        super().__init__()
        self.node = node
        self.publisher = self.node.create_publisher(AckermannDrive, 'drive', 1)
        self.ang = ang
        self.speed = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, ang, speed):
        with self.condition:
            self.ang = ang
            self.speed = speed
            self.condition.notify()

    def stop(self):
        with self.condition:
            self.done = True
            self.condition.notify()
        # Waiting for the thread to finish
        self.join()

    def run(self):
        drive = AckermannDrive()
        while True:
            with self.condition:
                if self.done:
                    break
                # Wait for a new message or timeout.
                self.condition.wait(self.timeout)
                # Copy state into twist message.
                drive.steering_angle = self.ang
                drive.speed = self.speed
            
            self.publisher.publish(drive)

        # Stop the car when Ctrl+C is pressed
        drive.steering_angle = self.ang
        drive.speed = 0
        self.publisher.publish(drive)


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    node = Node('teleop_node')
    settings = saveTerminalSettings()

    ang = 45 # starting angle
    speed = 0 # not moving
    pub_thread = PublishThread(node, rate=0.0, ang=ang)
    try:
        print(msg)
        pub_thread.update(ang, speed) # Starting position
        while True:
            key = getKey(settings, 0.1)
            if key in moveBindings:
                # range is -20 to 75
                if key == 'd' and ang > -20:
                    ang -= 5
                elif key == 'a' and ang < 75:
                    ang += 5
                elif key == 'w':
                    speed = 1
                elif key == 's':
                    speed = -1
                print("Angle: ", ang)
            else:
                if key == '\x03':  # Ctrl+C
                    break
            
            pub_thread.update(ang, speed)
            speed = 0

    except Exception as e:
        print(e)
    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
        node.destroy_node()

if __name__=="__main__":
    main()
