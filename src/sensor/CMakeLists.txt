cmake_minimum_required(VERSION 3.10)

project(servo_plugin)

# C++17 required for rclcpp and tf2_ros
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
)

# Link directories (for Gazebo)
link_directories(${GAZEBO_LIBRARY_DIRS})

# Plugin source
add_library(servo_plugin SHARED servo_plugin.cc)

# Link libraries (ROS2 + Gazebo)
target_link_libraries(servo_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
)

# Optional: install
install(TARGETS servo_plugin
  LIBRARY DESTINATION lib
)
