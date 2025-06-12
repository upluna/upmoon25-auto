#!/bin/bash

echo Cleaning stale build...
sudo rm -rf build/frontend build/backend install/frontend install/backend log/

echo Building upmoon25-auto packages...
colcon build --packages-select interfaces
colcon build --packages-select frontend backend

echo Building gazebo plugins...
sudo rm -rf src/sensor/build
mkdir src/sensor/build
cd src/sensor/build
cmake ..
make


echo "Install complete! Remember to source the setup file: source install/local_setup.bash"