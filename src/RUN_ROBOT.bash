#!/bin/bash
echo Building ROS2 Packages...
sudo colcon build
source install/local_setup.bash

# Kill child processes when the script is interrupted or exits
trap "echo 'Shutting down...'; kill 0" EXIT

echo Launching Nodes...
ros2 launch frontend jet_launch.py &
ros2 launch backend launch.py

# Wait for nodes to finish
wait