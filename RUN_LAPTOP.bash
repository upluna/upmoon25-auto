#!/bin/bash
echo Building ROS2 Packages...
sudo colcon build --packages-select frontend backend
source install/local_setup.bash

# Kill child processes when the script is interrupted or exits
trap "echo 'Shutting down...'; kill 0" EXIT

echo Launching Nodes...
rviz2 &
ros2 run frontend rgb_transport &
ros2 run frontend js_driver &
ros2 bag record /odom /camera /depth /points &

# Wait for nodes to finish
wait