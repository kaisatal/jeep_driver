#!/usr/bin/env python
import threading
import rospy
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
    def __init__(self, rate, ang):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('drive', AckermannDrive, queue_size = 1)

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
        self.condition.acquire()
        self.ang = ang
        self.speed = speed

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0)
        self.join()

    def run(self):
        drive = AckermannDrive()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            drive.steering_angle = self.ang
            drive.speed = self.speed

            self.condition.release()

            # Publish.
            self.publisher.publish(drive)

        # Publish stop message when thread exits.
        drive.steering_angle = 0.0
        drive.speed = 0.0
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

if __name__=="__main__":
    settings = saveTerminalSettings()
    rospy.init_node('teleop_node')
    ang = 45 # starting angle
    pub_thread = PublishThread(0.0, ang)
    speed = 0
    prev_key = None
    rospy.loginfo(msg)
    try:
        pub_thread.update(ang, speed)
        while(1):
            key = getKey(settings, 0.1)
            if key in moveBindings.keys():
                # range is -20 to 75
                if key == 'd' and ang > -20:
                    ang -= 5
                elif key == 'a' and ang < 75:
                    ang += 5
                elif key == 'w':
                    speed = 1
                elif key == 's':
                    speed = -1

                print("Angle:", ang)
                prev_key = key
            else:
                if key == '':
                    speed = 0
                if (key == '\x03'): # Ctrl+C
                    break
            pub_thread.update(ang, speed)
            speed = 0

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
