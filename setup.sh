#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source $HOME/go2-ros2/src/cyclonedds_ws/install/setup.bash
echo "Setup custom go2 ros2 environment"
source $HOME/go2-ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp0s31f6" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
