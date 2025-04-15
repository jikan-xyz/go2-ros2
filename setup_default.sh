#!/bin/bash
echo "Setup unitree ros2 environment with default interface"
source /opt/ros/humble/setup.bash
source $HOME/go2-ros2/src/cyclonedds_ws/install/setup.bash
echo "Setup custom go2 ros2 environment"
source $HOME/go2-ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
