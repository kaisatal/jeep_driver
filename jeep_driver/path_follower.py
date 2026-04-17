#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import math
import rosbag2_py

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

# Get path from file
def read_last_path(bag_uri):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_uri,
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()

    try:
        reader.open(storage_options, converter_options)
    except Exception:
        return None

    last_path = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/path':
            try:
                msg = deserialize_message(data, Path)
                last_path = msg
            except Exception:
                continue

    return last_path

# Alternative: manual path creation
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
        self.min_steering_deg = -20.0
        self.max_steering_deg = 75.0

        # State
        self.current_pose = None
        self.last_target_index = 0

        '''# Message from /path (nav_msgs/Path)
        self.path = read_last_path('last_path_bag')

        if self.path is None or len(self.path.poses) == 0:
            self.get_logger().error("No valid path loaded")
            self.path = None'''
        
        # Manual path creation
        self.path = Path()
        self.path.header.frame_id = "map"
        # Manual sample path
        coords = [
            (0.0, 0.0),
            (0.0, 1.0),
            (0.0, 2.0)
        ]
        self.path.poses = [make_pose(x, y) for x, y in coords]

        self.create_subscription(PoseWithCovarianceStamped, 'pcl_pose', self.pose_callback, 10)
        self.pub = self.create_publisher(AckermannDrive, 'path_drive', 10)
        self.create_timer(0.1, self.control_loop)  # 10 Hz

    def pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose

    def get_lookahead_point(self):
        if self.current_pose is None or self.path is None:
            return None
        
        poses = self.path.poses
        if len(poses) == 0:
            return None

        current_pos = self.current_pose.pose.position

        for i in range(self.last_target_index, len(poses)):
            point = poses[i].pose.position
            if distance(current_pos, point) >= self.lookahead_distance:
                self.last_target_index = i
                return point

        return poses[-1].pose.position

    def control_loop(self):
        if self.current_pose is None or self.path is None:
            return
        
        poses = self.path.poses
        if len(poses) == 0:
            return

        target = self.get_lookahead_point()
        if target is None:
            return

        pose = self.current_pose.pose
        position = pose.position

        goal = poses[-1].pose.position

        # Stop if very close to goal
        if distance(position, goal) < 1.0:
            self.get_logger().info("Reached end of Path")
            drive_msg = AckermannDrive()
            drive_msg.steering_angle = 0.0
            drive_msg.speed = 0.0
            self.pub.publish(drive_msg)
            raise KeyboardInterrupt # just needs to shut down

        # Get the vector
        dx = target.x - position.x
        dy = target.y - position.y

        yaw = yaw_from_quaternion(pose.orientation)

        # Rotate the vector to be in robot frame
        local_forward =  math.sin(yaw)*dx + math.cos(yaw)*dy # map has y as forward
        local_lateral = math.cos(yaw)*dx - math.sin(yaw)*dy
        
        if local_forward >= 0:
            speed = 1.0
        else:
            speed = -1.0

        curvature = 2.0 * local_lateral / (self.lookahead_distance ** 2)
        steering_rad = math.atan(self.wheelbase * curvature)
        steering_deg = math.degrees(steering_rad)

        # Clamp the steering angle
        steering_deg = max(self.min_steering_deg, min(self.max_steering_deg, steering_deg))

        drive_msg = AckermannDrive()
        drive_msg.steering_angle = steering_deg
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