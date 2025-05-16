from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontend',
            executable='drive_motors',
            name='drive_motors_node',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='arduino_driver',
            name='arduino_driver_node',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='rgb_driver',
            name='rgb_driver',
            output='screen'
        ),
    ])
