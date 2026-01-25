#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source Isaac Lab setup if available
if [ -f "${ISAACLAB_PATH}/isaaclab.sh" ]; then
    # Ensure the python alias is available
    export PATH="${ISAACLAB_PATH}/_isaac_sim:$PATH"
fi

# Install the project in editable mode if mapped
if [ -f "/workspace/tidybot_isaac/source/tidybot_isaac/setup.py" ]; then
    echo "Installing tidybot_isaac in editable mode..."
    ${ISAACLAB_PATH}/isaaclab.sh -p -m pip install -e /workspace/tidybot_isaac/source/tidybot_isaac
fi

# Configuration for FastDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml
export ROS_DOMAIN_ID=0

exec "$@"
