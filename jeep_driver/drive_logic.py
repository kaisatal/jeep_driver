#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32

class DriveLogicNode(Node):
    def __init__(self):
        # ROS2 node setup
        super().__init__('drive_logic_node')

        self.create_subscription(AckermannDrive, 'path_drive', self.path_callback, 10)
        self.create_subscription(AckermannDrive, 'keyboard_drive', self.keyboard_callback, 10)
        self.create_subscription(Int32, 'input_choice', self.input_choice_callback, 10)
        self.pub = self.create_publisher(AckermannDrive, 'cmd_drive', 10)
        
        # 1 - /keyboard_drive (manual driving), 2 - /path_drive (autonomous driving)
        self.input_choice = 1
        self.last_keyboard_msg = AckermannDrive()
        self.last_path_msg = AckermannDrive()
        
        # Timer for publishing output
        self.timer = self.create_timer(0.02, self.publish_cmd)  # 50 Hz
    
    def input_choice_callback(self, msg: Int32):
        self.input_choice = msg.data

    def keyboard_callback(self, msg: AckermannDrive):
        self.last_keyboard_msg = msg

    def path_callback(self, msg: AckermannDrive):
        self.last_path_msg = msg

    def publish_cmd(self):
        cmd = AckermannDrive()

        if self.input_choice == 1:
            cmd.steering_angle = self.last_keyboard_msg.steering_angle
            cmd.speed = self.last_keyboard_msg.speed

        elif self.input_choice == 2:
            cmd.steering_angle = self.last_path_msg.steering_angle
            cmd.speed = self.last_path_msg.speed

        else:
            # Fallback safety
            cmd.steering_angle = 45 # middle
            cmd.speed = 0.0

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = DriveLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Drive Logic node.")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
