from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_motion_routines',
            executable='ur_motion_routines_node',
            name='ur_motion_routines_node',
            output='screen',
        )
    ])