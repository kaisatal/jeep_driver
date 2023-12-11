#!/usr/bin/env python
import threading

import rospy

from ackermann_msgs.msg import AckermannDrive

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Reading from the keyboard  and Publishing to AckermannDrive!
---------------------------
Moving around:
   a         d

CTRL-C to quit
"""

moveBindings = {
        'a': 1,
        'd':-1,
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('drive', AckermannDrive, queue_size = 1)

        self.ang = 0.0
        
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, ang):
        self.condition.acquire()
        self.ang = ang
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0)
        self.join()

    def run(self):
        drive = AckermannDrive()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            drive.steering_angle = self.ang

            self.condition.release()

            # Publish.
            rospy.loginfo(drive.steering_angle)
            self.publisher.publish(drive)

        # Publish stop message when thread exits.
        drive.steering_angle = 0.0
        self.publisher.publish(drive)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    settings = saveTerminalSettings()
    rospy.init_node('teleop_node')
    pub_thread = PublishThread(0.0)
    ang = 0
    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(ang)

        print(msg)
        while(1):
            key = getKey(settings, 0.5)
            if key in moveBindings.keys():
                if key == 'a' and ang > -100:
                    ang -= 1
                if key == 'd' and ang < 100:
                    ang += 1
                    
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and ang == 0:
                    continue
                # ang = 0
                if (key == '\x03'):
                    break

            pub_thread.update(ang)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)