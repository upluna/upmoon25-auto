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

    # TODO this is a really stupid way of doing this

    pkg_name = 'backend'
    file_subpath = 'description/robot.urdf.xacro'

    FSM_FILE = '/home/max/Documents/robotics/upmoon25-auto/src/backend/resource/fsm.txt'
    NAV2_CONFIG = '/home/max/Documents/robotics/sim/src/sim/config/nav2_config.yaml'


    xacro_file = "/home/upmoon25/ros2/upmoon25-auto/src/backend/description/robot.urdf.xacro"
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
        'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )
   
    rgb_driver = Node(
        package='frontend',
        executable='rgb_driver',
        output='screen',
        parameters=[{
            'publish_raw' : False,
            'publish_compressed' : True
        }]
    )

    t265_driver = Node(
        package='frontend',
        executable='t265_driver',
        output='screen',
    )

    tag_detector = Node(
        package='frontend',
        executable='tag_detector',
        output='screen',
    )

    arduino_driver = Node(
        package='frontend',
        executable='arduino_driver',
        name='arduino_driver_node',
        output='screen'
    )

    joint_state = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    return launch.LaunchDescription([
        node_robot_state_publisher,
        tag_detector,
        rgb_driver,
        t265_driver,
        #arduino_driver,
        #joint_state
    ])
