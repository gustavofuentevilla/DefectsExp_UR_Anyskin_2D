from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch anyskin_sensor_publisher node
        Node(
            package='anyskin_sensor_publisher',
            executable='anyskin_sensor_node',
            name='anyskin_sensor_publisher',
            output='screen',
        )
    ])