#!/bin/bash

# Kill child processes when the script is interrupted or exits
trap "echo 'Shutting down...'; kill 0" EXIT
source install/local_setup.bash
echo Launching Nodes...
rviz2 &
ros2 run frontend rgb_transport &

if [ "$1" = "record" ]; then
    ros2 bag record /odom /camera /depth /points &
fi

gnome-terminal -- bash -c "ros2 run frontend js_driver;" 

# Wait for nodes to finish
wait