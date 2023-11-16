#!/bin/bash

# Replace 'ros2-humble' with the actual folder name where your ROS 2 is installed
ROS2_INSTALL_DIR="/opt/ros/humble/"

# Check if the directory exists
if [ -d "$ROS2_INSTALL_DIR" ]; then
    # Source the setup.bash script
    source "$ROS2_INSTALL_DIR"setup.bash
    echo "Sourced ROS 2 setup.bash from $ROS2_INSTALL_DIR"

		cd /home/ffi3/ffi3_intp_ROS2Worm || exit
		colcon build

		if [ -d "/home/ffi3/ffi3_intp_ROS2Worm/install/" ]; then
			source /home/ffi3/ffi3_intp_ROS2Worm/install/setup.bash
			ros2 run ros2_worm_multiplayer worm_grid_node
		fi
else
    echo "Error: ROS 2 installation directory not found: $ROS2_INSTALL_DIR"
fi