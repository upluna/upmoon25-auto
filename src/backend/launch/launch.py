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


    xacro_file = "/home/max/Documents/robotics/upmoon25-auto/src/backend/description/robot.urdf.xacro"
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
        'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )
    
 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
 
 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')    
    
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'map', 'odom'
        ]
    )

    mapper = Node(
        package='backend',
        executable='global_mapper',
        output='screen'
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

    # NAV2
    nav2 = launch.LaunchDescription([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[NAV2_CONFIG]
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[NAV2_CONFIG]
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[NAV2_CONFIG]
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[NAV2_CONFIG]
        ),
        Node(
            package="nav2_recoveries",
            executable="recoveries_server",
            name="recoveries_server",
            output="screen",
            parameters=[NAV2_CONFIG]
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "recoveries_server",
                    "bt_navigator",
                    "waypoint_follower"
                ]
            }])
        ])

    return launch.LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        map_odom_tf,
        mapper,
        costmapper,
        path_planner,
        motion_controller
    ])
