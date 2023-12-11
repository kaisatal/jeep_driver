import rospy
from ackermann_msgs.msg import AckermannDrive

def main():
    pub = rospy.Publisher('drive', AckermannDrive, queue_size=10)
    rospy.init_node('teleop_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    msg = AckermannDrive()
    msg.steering_angle = 0
    msg.speed = 0
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass