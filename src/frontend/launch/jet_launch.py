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
        Node(
            package='frontend',
            executable='conveyor',
            name='conveyor',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='bucket_spin',
            name='bucket_spin',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='t265_driver',
            name='t265_driver',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='depth_driver',
            name='depth_driver',
            output='screen'
        ),
    ])
