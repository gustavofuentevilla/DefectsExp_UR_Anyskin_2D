from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sync_data',
            executable='sync_data',
            name='sync_node',
            output='screen',
        )
    ])
