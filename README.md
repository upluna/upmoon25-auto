# UPR Lunar Construction Robot - Repository Guide

This repository contains both the ROS2 code and a guide for connecting to and operating the UPR lunar construction robot. It is *highly* recommended to read the documentation at https://docs.ros.org/en/humble/Tutorials.html (at least up to and including the header *Writing a simple service and client*) before continuing.

## Setup
### Connecting to the Jetson
- Ensure that the Jetson and your device are connected to the same network. This network must also be one in which the two devices can see (ping) each other. By default, the Jetson contains a startup script which will automatically connect it to the router's network (SSID `Team_30`, password `UPRobotics!`). 
- Now you can SSH into the Jetson from your device:
	- Acquire the Jetson's IP address. The Jetson is configured to use the static IP address `192.168.0.2` when connected to the the router. If It's not on the router, you'll have to acquire its IPV4 address by typing `iwconfig -a` into the console on the Jetson.
	- On Ubuntu, type `ssh upmoon25@192.168.0.2` into the console. The password is `upmoon25`. 
	- On Windows, you will have to do a series of steps; here's how:
   		- Open up Windows Powershell and Run as Administrator
		- You will need access to OpenSSH Client and OpenSSH Server:
    			- It should pop up `PS C:\WINDOWS\system32` as the home directory
  			- Type in `Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0`
			- Type in `Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0`
			- To check if both Client and Server are Present, type `Get-WindowsCapability -Online | Where-Object Name -Like 'OpenSSH*'`
			- After the Client and Server are present, we want to start and set the Service! Run `Start-Service sshd`, then `Set-Service -Name sshd -StartupType 'Automatic'`.
			- We can check if the Service is running by typing `Get-Service sshd`!
     		- Finally, type `ssh upmoon25@192.168.0.2` into powershell. The password is `upmoon25`.
### Running the code in this repository

This project is designed to be run on three machines simulataneously; the Jetson, a laptop for RC control, and a laptop for autonomous control. Ensure ROS2 Humble is installed on the machine you're running the code on. Of course, the Jetson already has ROS2 Humble setup. There's an installation tutorial [here](https://docs.ros.org/en/humble/Installation.html). **Make sure your domain ID is set to 8888!** 

#### Jetson:

`cd` into the root of the workspace (the folder containing all the packages). For the Jetson this should be `ros2/upmoon25-auto`. In your console, type the following commands:
1. `./BUILD.bash` - Builds the ROS2 packages for the project
2. `source install/local_setup.bash` - Sets the ROS2 environment variables needed for running the project
3. `./RUN_ROBOT.bash` - Starts all of the nodes for controlling the robot hardware (cameras, motors, servos, etc.). **At this point the robot should be considered live!** 

#### RC Laptop:
Plug the joystick controller into the laptop. `cd` into the root workspace (`.../upmoon25-auto`). In your console type the following commands:
1. `./INSTALL.bash` - This script only needs to be run once. The following commands need to be ran everytime you make changes to the project.
2. `./BUILD.bash`
3. `source install/local_setup.bash`
4. `./RUN_LAPTOP_RC.bash` - This launches **rviz2** and the joystick control node.
    - Note: running `./RUN_LAPTOP_RC.bash record` will start a **rosbag** recording of the camera data along with launching the other nodes.
5. The robot is now live and can be controlled using the joystick:
![image](https://github.com/user-attachments/assets/cd5c4311-f8fa-4ea6-933d-198a95b25a23)

#### Autonomy Laptop:
`cd` into the root workspace (`.../upmoon25-auto`). In your console type the following commands:
1. `./BUILD.bash`
2. `source install/local_setup.bash`
3. `./MAX_RUN.sh`

## Software Architecture
### Overview
The workspace is split into four packages:
* *backend* - contains all code relevant to autonomous and semi-autonomous vehicle operations (path planning, automatic mining, motion planning, obstacle detection, behavior, remote control)
* *frontend* - contains all code relevant to interacting with the vehicle hardware (interfacing with the arduino, controlling servos and motors).
* *interfaces* - defines custom ROS2 messages and services which are used by nodes in either the frontend or backend
* *sensor* - defines custom plugins for use in Gazebo, such as servo_plugin

The backend communicates with the hardware via the frontend, which provides an abstraction layer that makes it easy to control the various components on the robot. For example - if a node in the backend (like `motion_controller`) wanted to make the robot move forward, it only needs to publish the appropriate message to the `/cmd_vel` topic. More details about the topics and nodes can be found in the source code.

### Simulation
The simulator, Gazebo, provides a very useful method for testing the autonomy system. Any developer working on the autonomy can use the simulator in order to accelerate development time, test aspects of the system in isolation, and work on the system while the physical robot is unavailable. The file `backend/launch/sim_launch.py` is an example launch file which will launch part the simulator. The `rc_controller` node should be started separately - this allows you to control the simulator robot.

Additionally, the `gz_worlds` folder in the root directory contains worlds for the Gazebo simulator. You might find these useful - `arena1.world` is a 1:1 recreation of the Artemis arena, and contains rough terrain with craters and rocks.

If you want to launch with a world, include the parameter `world:="path to world"`, i.e.,

`ros2 launch backend launch.py world:=~/robotics/upmoon25/gz_worlds/arena1.world`

If you want to launch headless (no GUI), include the parameter `gui:=false`
