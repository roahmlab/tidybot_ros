#!/bin/bash
#
# Entrypoint for Isaac Sim with ROS 2 Jazzy
# Uses Isaac Sim's internal ROS 2 libraries (Python 3.11 compatible)
#

# Use Isaac Sim's internal ROS 2 libraries
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib"

# Configure DDS
if [ -f /fastdds.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml
fi

# Execute the command passed to the container
exec "$@"