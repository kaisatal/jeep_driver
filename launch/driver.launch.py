from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jeep_driver',
            executable='driver.py',
            name='driver_node',
            output='screen'
        ),
        Node(
            package='jeep_driver',
            executable='mrp_sensor.py',
            name='feedback_node',
            output='screen'
        )
    ])