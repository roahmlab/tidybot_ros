#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source Isaac Lab setup if available
if [ -f "${ISAACLAB_PATH}/isaaclab.sh" ]; then
    # Ensure the python alias is available
    export PATH="${ISAACLAB_PATH}/_isaac_sim:$PATH"
fi

# Configuration for FastDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml
export ROS_DOMAIN_ID=0

exec "$@"
