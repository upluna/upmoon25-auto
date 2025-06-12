#!/bin/bash

colcon build --packages-select backend frontend

source install/local_setup.bash

rviz2 &
ros2 run backend rgb_transport &
ros2 run backend main_controller
