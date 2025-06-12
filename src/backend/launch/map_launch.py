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
    
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'map', 'odom'
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )

    mapper = Node(
        package='backend',
        executable='global_mapper',
        output='screen',
        parameters=[{
            'use_sim_data': False,
            'map_on': True,
            'use_sim_time': True
        }]
    )

    costmapper = Node(
        package='backend',
        executable='global_costmapper',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return launch.LaunchDescription([
        map_odom_tf,
        mapper,
        costmapper,
    ])
