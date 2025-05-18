#!/bin/bash

echo Cleaning stale build...
sudo rm -rf build/frontend build/backend install/frontend install/backend log/

echo Building upmoon25-auto packages...
sudo colcon build --packages-select frontend backend
source install/local_setup.bash
