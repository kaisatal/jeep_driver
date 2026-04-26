#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.serialization
from nav_msgs.msg import Path
import rosbag2_py
import os
import shutil

class LastPathRecorderNode(Node):
    def __init__(self):
        super().__init__('last_path_recorder_node')
        self.last_msg = None
        self.create_subscription(Path, '/path', self.path_callback, 10)

    def path_callback(self, msg):
        self.last_msg = msg

    def write_last(self):
        if self.last_msg is None:
            return
        
        bag_path = 'last_path_bag'
        
        # Remove previous existing file
        if os.path.exists(bag_path):
            shutil.rmtree(bag_path)

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            name='/path',
            type='nav_msgs/msg/Path',
            serialization_format='cdr'
        )
        writer.create_topic(topic_info)

        serialized = rclpy.serialization.serialize_message(self.last_msg)
        writer.write('/path', serialized, self.get_clock().now().nanoseconds)

def main():
    rclpy.init()
    node = LastPathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_last()
        node.get_logger().info("Written to last_path_bag")
    finally:
        node.destroy_node()