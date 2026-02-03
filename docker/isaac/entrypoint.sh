#!/bin/bash
set -e

# Source ROS 2
# NOTE: We do NOT source /opt/ros/jazzy/setup.bash because it appends Python 3.12 paths 
# to PYTHONPATH, which causes crashes in Isaac Sim (Python 3.11).
# instead we manually configure the environment below.

# -----------------------------------------------------------------------------
# Isaac Lab Setup
# -----------------------------------------------------------------------------
# Source Isaac Lab setup if available (ISAACLAB_PATH is set in base image)
if [ -n "$ISAACLAB_PATH" ] && [ -f "${ISAACLAB_PATH}/isaaclab.sh" ]; then
    # Ensure the python alias is available
    export PATH="${ISAACLAB_PATH}/_isaac_sim:$PATH"
fi

# -----------------------------------------------------------------------------
# Install Project
# -----------------------------------------------------------------------------
# Install the project in editable mode if mapped
if [ -f "/workspace/tidybot_isaac/source/tidybot_isaac/setup.py" ]; then
    echo "Installing tidybot_isaac in editable mode..."
    # Check if isaaclab.sh exists, otherwise try python directly if simpler
    if [ -x "${ISAACLAB_PATH}/isaaclab.sh" ]; then
        ${ISAACLAB_PATH}/isaaclab.sh -p -m pip install -e /workspace/tidybot_isaac/source/tidybot_isaac
    else
        # Fallback if specific script unavailable (though typical in isaac-lab-base)
        echo "Warning: isaaclab.sh not found, skipping editable install."
    fi
fi

# -----------------------------------------------------------------------------
# ROS 2 Middleware Configuration
# -----------------------------------------------------------------------------
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml
export ROS_DOMAIN_ID=0
# Add workspace src to ROS_PACKAGE_PATH/COLCON_PREFIX_PATH to resolve package:// URIs
export ROS_PACKAGE_PATH=/workspace/src:$ROS_PACKAGE_PATH

# Add Internal ROS Bridge to LD_LIBRARY_PATH (from isaac-sim-ros2/entrypoint.sh)
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib"

# Add alias to run Isaac Sim with ROS Control extension enabled
if ! grep -q "alias run_sim" /root/.bashrc; then
    echo "alias run_sim='/isaac-sim/isaac-sim.sh --/isaac/startup/ros_sim_control_extension=True'" >> /root/.bashrc
fi

exec "$@"
