#!/bin/bash

sudo colcon build --packages-select backend frontend

source install/local_setup.bash

rviz2 &
ros2 run frontend rgb_transport &
ros2 run backend main_controller
