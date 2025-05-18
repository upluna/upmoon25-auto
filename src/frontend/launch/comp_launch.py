from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro


xacro_file = "/home/upmoon25/ros2/upmoon25-auto/src/backend/description/robot.urdf.xacro"
robot_description_raw = xacro.process_file(xacro_file).toxml()

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
            output='screen',
            parameters=[{
                'publish_raw' : False,
                'publish_compressed' : True
            }]
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
            output='screen',
            parameters=[{
                'demand_publish' : True
            }]
        ),
        Node(
            package='backend',
            executable='mining_controller',
            name='mining_controller',
            output='screen'
        ),
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
        'robot_description': robot_description_raw,
        'use_sim_time': True}]
        ),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'map', 'odom'
        ]
        ),
    ])
