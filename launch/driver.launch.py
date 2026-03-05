from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jeep_driver',
            executable='driver_node',
            name='driver_node',
            output='screen'
        ),
        Node(
            package='jeep_driver',
            executable='feedback_node',
            name='feedback_node',
            output='screen'
        )
    ])