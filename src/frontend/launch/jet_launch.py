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
            executable='conveyor',
            name='conveyor_node',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='bucket_servos',
            name='bucket_servos_node',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='camera',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='bucket_spin',
            name='bucket_spin_node',
            output='screen'
        ),
        Node(
            package='frontend',
            executable='camera_pan',
            name='camera_pan_node',
            output='screen'
        ),
        # Node(
        #     package='frontend',
        #     executable='arduino_driver',
        #     name='arduino_driver_node',
        #     output='screen'
        # ),
    ])
