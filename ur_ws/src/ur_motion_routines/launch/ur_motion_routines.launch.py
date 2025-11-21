from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # declare a launch argument 'i' so the user can pass it on the command line
    iteration = DeclareLaunchArgument(
        'i', default_value='1', description='Iteration parameter for the robot motion routine')

    i_cfg = LaunchConfiguration('i')

    return LaunchDescription([
        iteration,
        Node(
            package='ur_motion_routines',
            executable='ur_motion_routines_node',
            name='ur_motion_routines_node',
            output='screen',
            parameters=[{'i': i_cfg}],
        )
    ])