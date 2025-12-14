#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CONTAINER_NAME=isaac_sim_tidybot
CACHE_DIR=$HOME/docker/isaac-sim

# Create cache directories (matching the working structure from README)
mkdir -p $CACHE_DIR/cache/{kit,ov,pip,glcache,computecache}
mkdir -p $CACHE_DIR/{config,data,logs,pkg,documents}

# Create Isaac Sim workspace with proper permissions
ISAAC_WORKSPACE=$SCRIPT_DIR/../isaac_sim_workspace
mkdir -p $ISAAC_WORKSPACE

# Ensure proper permissions for container access
chmod -R 777 $CACHE_DIR $ISAAC_WORKSPACE

# X11 forwarding setup
xhost +local:

# Resolve host group IDs for reliable permission mapping inside container
RENDER_GID=$(getent group render | cut -d: -f3 || true)
VIDEO_GID=$(getent group video | cut -d: -f3 || true)

DOCKER_OPTIONS=""
DOCKER_OPTIONS+="--name $CONTAINER_NAME "
DOCKER_OPTIONS+="-it "
DOCKER_OPTIONS+="--privileged "   
DOCKER_OPTIONS+="--gpus all "
DOCKER_OPTIONS+="--network=host "

# Resource limits (matching working config from README)
DOCKER_OPTIONS+="--ulimit nofile=65535:65535 "
DOCKER_OPTIONS+="--ulimit memlock=-1 "
DOCKER_OPTIONS+="--ulimit stack=67108864 "

DOCKER_OPTIONS+="-e ACCEPT_EULA=Y "
DOCKER_OPTIONS+="-e PRIVACY_CONSENT=Y "

# Critical NVIDIA environment variables
DOCKER_OPTIONS+="-e NVIDIA_VISIBLE_DEVICES=all "
DOCKER_OPTIONS+="-e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display "
DOCKER_OPTIONS+="-e __GLX_VENDOR_LIBRARY_NAME=nvidia "

# X11 display forwarding (matching working config from README)
DOCKER_OPTIONS+="-e DISPLAY=$DISPLAY "
DOCKER_OPTIONS+="-v /tmp/.X11-unix:/tmp/.X11-unix:rw "
DOCKER_OPTIONS+="-v $HOME/.Xauthority:/root/.Xauthority:rw "

# Device access for GPU rendering
DOCKER_OPTIONS+="--device /dev/dri:/dev/dri "

# Add video/render groups for device permissions
if [ -n "$VIDEO_GID" ]; then DOCKER_OPTIONS+="--group-add $VIDEO_GID "; fi
if [ -n "$RENDER_GID" ]; then DOCKER_OPTIONS+="--group-add $RENDER_GID "; fi

# Mount Vulkan ICD configuration
if [ -d "/usr/share/vulkan/icd.d" ]; then
    DOCKER_OPTIONS+="-v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro "
    DOCKER_OPTIONS+="-e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json "
fi

# Mount EGL vendor libraries
if [ -d "/usr/share/glvnd/egl_vendor.d" ]; then
    DOCKER_OPTIONS+="-v /usr/share/glvnd/egl_vendor.d:/usr/share/glvnd/egl_vendor.d:ro "
    DOCKER_OPTIONS+="-e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json "
fi

# Isaac Sim cache mounts
DOCKER_OPTIONS+="-v $CACHE_DIR/cache/kit:/isaac-sim/kit/cache/Kit:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/cache/ov:/root/.cache/ov:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/cache/pip:/root/.cache/pip:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/cache/glcache:/root/.cache/nvidia/GLCache:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/cache/computecache:/root/.nv/ComputeCache:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/logs:/root/.nvidia-omniverse/logs:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/config:/root/.nvidia-omniverse/config:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/data:/root/.local/share/ov/data:rw "
DOCKER_OPTIONS+="-v $ISAAC_WORKSPACE:/isaac_workspace:rw "
DOCKER_OPTIONS+="-v $CACHE_DIR/documents:/root/Documents:rw "

# Mount TidyBot workspace for URDF access
DOCKER_OPTIONS+="-v $SCRIPT_DIR/../src/tidybot_description:/tidybot_description:rw "

# ROS 2 DDS configuration (must match tidybot container)
DOCKER_OPTIONS+="-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} "
DOCKER_OPTIONS+="-e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml "
DOCKER_OPTIONS+="-v $SCRIPT_DIR/fastdds_isaac.xml:/fastdds.xml:ro "

# Parse arguments
MODE="gui"  # Default to GUI mode
if [ "${2:-""}" == "shell" ]; then
    MODE="shell"
fi

if [ "${1:-""}" == "restart" ]; then
    docker rm -f $CONTAINER_NAME 2>/dev/null
    if [ "$MODE" == "shell" ]; then
        echo "Starting Isaac Sim container with bash shell..."
        docker run --rm --entrypoint bash $DOCKER_OPTIONS nvcr.io/nvidia/isaac-sim:5.1.0
    else
        echo "Starting Isaac Sim GUI..."
        docker run --rm --entrypoint bash $DOCKER_OPTIONS nvcr.io/nvidia/isaac-sim:5.1.0 \
            -c "./isaac-sim.sh"
    fi
elif [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        docker start -ai $CONTAINER_NAME
    else
        if [ "$MODE" == "shell" ]; then
            echo "Starting Isaac Sim container with bash shell..."
            docker run --rm --entrypoint bash $DOCKER_OPTIONS nvcr.io/nvidia/isaac-sim:5.1.0
        else
            echo "Starting Isaac Sim GUI..."
            docker run --rm --entrypoint bash $DOCKER_OPTIONS nvcr.io/nvidia/isaac-sim:5.1.0 \
                -c "./isaac-sim.sh"
        fi
    fi
else
    docker exec -it $CONTAINER_NAME /bin/bash
fi