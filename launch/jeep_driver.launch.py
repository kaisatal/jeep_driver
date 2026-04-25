from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jeep_driver',
            executable='jeep_driver_node',
            name='jeep_driver_node',
            output='screen'
        ),
        Node(
            package='jeep_driver',
            executable='feedback_node',
            name='feedback_node',
            output='screen'
        ),
        Node(
            package='jeep_driver',
            executable='drive_logic_node',
            name='drive_logic_node',
            output='screen'
        )
    ])