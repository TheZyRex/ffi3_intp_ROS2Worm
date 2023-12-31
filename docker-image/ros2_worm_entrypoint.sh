#!/bin/bash

# Replace 'ros2-humble' with the actual folder name where your ROS 2 is installed
ROS2_INSTALL_DIR="/opt/ros/humble/"
ROS2_WORM_DIR="/home/ffi3/ffi3_intp_ROS2Worm/"

# Check if the directory exists
if [ -d "$ROS2_INSTALL_DIR" ]; then
    # Source the setup.bash script
    source "$ROS2_INSTALL_DIR"setup.bash || {
			echo "Error: Sorcing ROS 2 setup.bash failed"
			exit 1
		}
    echo "Sourced ROS 2 setup.bash from $ROS2_INSTALL_DIR"

		# Change in the project directory
		cd $ROS2_WORM_DIR || exit 1

		# pull the lastest version
		git pull || { 
			echo "Error: Git pull failed" 
		}

		echo "Buildind worm package - can take upto 2hrs... be patient"

		# build the ros2 package
		colcon build || {
			echo "Error: build of ros2_worm_multiplayer failed"
			exit 1
		}

		# if build was successfull and install dir exists, source the directory
		if [ -d "/home/ffi3/ffi3_intp_ROS2Worm/install/" ]; then
			source /home/ffi3/ffi3_intp_ROS2Worm/install/setup.bash || {
				echo "Error: Sorcing ROS2Worm setup.bash failed"
				exit 1
			}

			echo "Starting worm_grid_node for $WORM_MIN_PLAYERS players to join"
			ros2 run ros2_worm_multiplayer worm_grid_node --ros-args -p numPlayers:=$WORM_MIN_PLAYERS
		fi
else
    echo "Error: ROS 2 installation directory not found: $ROS2_INSTALL_DIR"
fi