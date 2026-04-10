from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jeep_driver',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),
        Node(
            package='jeep_driver',
            executable='drive_logic_node',
            name='drive_logic_node',
            output='screen'
        )
    ])