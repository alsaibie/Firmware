#!/bin/bash
#source /home/alsaibie/UWSim/install_isolated/setup.bash
source /home/alsaibie/catkin_ws/devel/setup.bash

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

