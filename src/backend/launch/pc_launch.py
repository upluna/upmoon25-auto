import launch
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


from launch_ros.actions import Node

# ros2 launch sim launch.py world:=~/Documents/robotics/gz_worlds/arena1.world gui:=false

def generate_launch_description():

    robot_file = 'robot.urdf.xacro'

    xacro_file = os.path.join(get_package_share_directory('backend'), f'description/{robot_file}')

    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
        'robot_description': robot_description_raw,
        'use_sim_time': False}]
    )
    
    
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'map', 'odom'
        ]
    )

    odom_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'odom', 'base_link'
        ]
    )

    depth_driver = Node(
        package='frontend',
        executable='depth_driver',
        output='screen'
    )

    mapper = Node(
        package='backend',
        executable='global_mapper',
        output='screen',
        parameters=[{
            'use_sim_data': False
        }]
    )

    costmapper = Node(
        package='backend',
        executable='global_costmapper',
        output='screen',
    )

    path_planner = Node(
        package='backend',
        executable='path_planner',
        output='screen',
    )

    motion_controller = Node(
        package='backend',
        executable='motion_controller',
        output='screen',
    )

    return launch.LaunchDescription([
        depth_driver,
        node_robot_state_publisher,
        map_odom_tf,
        odom_base_tf,
        mapper,
        #costmapper
    ])
