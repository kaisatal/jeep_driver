#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def distance(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    return math.hypot(dx, dy)

# For manual path creation
def make_pose(x, y, yaw=0.0, frame_id="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    # yaw to quaternion (z-axis rotation)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    
    return pose

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Parameters
        self.lookahead_distance = 0.8 # meters
        self.wheelbase = 0.6 # meters

        # Magnet value (angle) range depends on how the magnet is situated, so this range might change
        self.min_steering_deg = -20
        self.max_steering_deg = 75

        # State
        self.current_pose = None

        # Manual path creation
        self.path = Path()
        self.path.header.frame_id = "map"
        # Sample path: forward 3 m, right 2 m
        coords = [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0),
            (3.0, -1.0),
            (3.0, -2.0),
        ]
        self.path.poses = [make_pose(x, y) for x, y in coords]

        self.last_target_index = 0
        self.create_subscription(PoseStamped, 'pcl_pose', self.pose_callback, 10)
        self.pub = self.create_publisher(AckermannDrive, 'path_drive', 10)
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_lookahead_point(self):
        if not self.current_pose or not self.path:
            return None

        current_pos = self.current_pose.pose.position

        for i in range(self.last_target_index, len(self.path)):
            point = self.path[i].pose.position
            if distance(current_pos, point) >= self.lookahead_distance:
                self.last_target_index = i
                return point

        return self.path[-1].pose.position

    def control_loop(self):
        if self.current_pose is None or not self.path:
            return

        target = self.get_lookahead_point()
        if target is None:
            return

        pose = self.current_pose.pose
        position = pose.position

        # Stop if very close to goal
        if distance(position, self.path[-1].pose.position) < 0.1:
            drive_msg = AckermannDrive()
            drive_msg.steering_angle = 0.0
            drive_msg.speed = 0.0
            self.pub.publish(drive_msg)
            return

        # Get the vector
        dx = target.x - position.x
        dy = target.y - position.y

        yaw = yaw_from_quaternion(pose.orientation)

        # Rotate the vector to be in robot frame
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy # local_x: front when positive, back when negative
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy # local_y: left when positive, right when negative
        
        if local_x >= 0:
            speed = 1.0
        else:
            speed = -1.0

        curvature = 2.0 * local_y / (self.lookahead_distance ** 2)
        steering_rad = math.atan(self.wheelbase * curvature)
        steering_angle_deg = math.degrees(steering_rad)

        # Clamp the steering angle
        steering_angle_deg = max(self.min_steering_deg, min(self.max_steering_deg, steering_angle_deg))

        drive_msg = AckermannDrive()
        drive_msg.steering_angle = steering_angle_deg
        drive_msg.speed = speed

        self.pub.publish(drive_msg)


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Path Follower node.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()