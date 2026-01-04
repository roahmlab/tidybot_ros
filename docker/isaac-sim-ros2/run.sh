#!/usr/bin/env bash
#
# Run Isaac Sim 5.1.0 with ROS 2 Jazzy support
# Enables ROS 2 communication with TidyBot container via FastDDS
#

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DOCKER_DIR=$SCRIPT_DIR/..
REPO_ROOT=$SCRIPT_DIR/../..

CONTAINER_NAME=isaac_sim_tidybot
IMAGE_NAME=isaac-sim-ros2:5.1.0
CACHE_DIR=$HOME/docker/isaac-sim

# =============================================================================
# Build custom Isaac Sim + ROS 2 image if needed
# =============================================================================
build_image() {
    echo "Checking for Isaac Sim + ROS 2 image..."
    if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
        echo "Building Isaac Sim + ROS 2 image (this may take a while)..."
        $SCRIPT_DIR/build.sh
        if [ $? -ne 0 ]; then
            exit 1
        fi
    else
        echo "Image $IMAGE_NAME already exists."
    fi
}

# =============================================================================
# Setup directories and permissions
# =============================================================================
setup_directories() {
    # Create cache directories
    mkdir -p $CACHE_DIR/cache/{kit,ov,pip,glcache,computecache}
    mkdir -p $CACHE_DIR/{config,data,logs,pkg,documents}

    # Create Isaac Sim workspace
    ISAAC_WORKSPACE=$REPO_ROOT/isaac_sim_workspace
    mkdir -p $ISAAC_WORKSPACE

    # Ensure proper permissions for container access
    chmod -R 777 $CACHE_DIR $ISAAC_WORKSPACE 2>/dev/null || true
}

# =============================================================================
# Build Docker options
# =============================================================================
build_docker_options() {
    # X11 forwarding setup
    xhost +local: > /dev/null 2>&1

    # Resolve host group IDs
    RENDER_GID=$(getent group render 2>/dev/null | cut -d: -f3 || true)
    VIDEO_GID=$(getent group video 2>/dev/null | cut -d: -f3 || true)

    DOCKER_OPTIONS=""
    DOCKER_OPTIONS+="--name $CONTAINER_NAME "
    DOCKER_OPTIONS+="-it "
    DOCKER_OPTIONS+="--privileged "   
    DOCKER_OPTIONS+="--gpus all "
    DOCKER_OPTIONS+="--network=host "

    # Resource limits
    DOCKER_OPTIONS+="--ulimit nofile=65535:65535 "
    DOCKER_OPTIONS+="--ulimit memlock=-1 "
    DOCKER_OPTIONS+="--ulimit stack=67108864 "
    DOCKER_OPTIONS+="--shm-size=16g "

    # EULA acceptance and root permission
    DOCKER_OPTIONS+="-e ACCEPT_EULA=Y "
    DOCKER_OPTIONS+="-e PRIVACY_CONSENT=Y "
    DOCKER_OPTIONS+="-e OMNI_KIT_ALLOW_ROOT=1 "

    # NVIDIA environment variables
    DOCKER_OPTIONS+="-e NVIDIA_VISIBLE_DEVICES=all "
    DOCKER_OPTIONS+="-e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display "
    DOCKER_OPTIONS+="-e __GLX_VENDOR_LIBRARY_NAME=nvidia "

    # X11 display forwarding
    DOCKER_OPTIONS+="-e DISPLAY=$DISPLAY "
    DOCKER_OPTIONS+="-v /tmp/.X11-unix:/tmp/.X11-unix:rw "
    DOCKER_OPTIONS+="-v $HOME/.Xauthority:/root/.Xauthority:rw "

    # Device access for GPU rendering
    DOCKER_OPTIONS+="--device /dev/dri:/dev/dri "

    # Add video/render groups
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
    DOCKER_OPTIONS+="-v $REPO_ROOT/src/tidybot_description:/tidybot_description:rw "

    # ROS 2 DDS configuration
    DOCKER_OPTIONS+="-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} "
    DOCKER_OPTIONS+="-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp "
    DOCKER_OPTIONS+="-e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml "
    DOCKER_OPTIONS+="-v $DOCKER_DIR/fastdds.xml:/fastdds.xml:ro "

    echo "$DOCKER_OPTIONS"
}

# =============================================================================
# Main
# =============================================================================
print_usage() {
    echo "Usage: $0 [command] [mode]"
    echo ""
    echo "Commands:"
    echo "  (none)    Start or attach to container"
    echo "  restart   Force restart the container"
    echo "  build     Build/rebuild the Docker image"
    echo ""
    echo "Modes:"
    echo "  gui       Run Isaac Sim GUI (default)"
    echo "  shell     Run bash shell only"
    echo ""
    echo "Examples:"
    echo "  $0                  # Start Isaac Sim GUI"
    echo "  $0 restart shell    # Restart with shell only"
    echo "  $0 build            # Rebuild the Docker image"
}

# Parse arguments
COMMAND="${1:-run}"
MODE="${2:-gui}"

if [ "$COMMAND" == "-h" ] || [ "$COMMAND" == "--help" ]; then
    print_usage
    exit 0
fi

if [ "$COMMAND" == "build" ]; then
    echo "Force rebuilding Isaac Sim + ROS 2 image..."
    docker rmi $IMAGE_NAME 2>/dev/null || true
    $SCRIPT_DIR/build.sh
    exit $?
fi

# Build image if needed
build_image

# Setup directories
setup_directories

# Get Docker options
DOCKER_OPTIONS=$(build_docker_options)

# Run container
if [ "$COMMAND" == "restart" ]; then
    docker rm -f $CONTAINER_NAME 2>/dev/null
    if [ "$MODE" == "shell" ]; then
        echo "Starting Isaac Sim container with bash shell..."
        echo "ROS 2 Jazzy is available. Run: ros2 topic list"
        docker run --rm $DOCKER_OPTIONS $IMAGE_NAME bash
    else
        echo "Starting Isaac Sim GUI with ROS 2 Simulation Control support..."
        docker run --rm $DOCKER_OPTIONS $IMAGE_NAME ./isaac-sim.sh --/isaac/startup/ros_sim_control_extension=True
    fi
elif [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        echo "Attaching to stopped container..."
        docker start -ai $CONTAINER_NAME
    else
        if [ "$MODE" == "shell" ]; then
            echo "Starting Isaac Sim container with bash shell..."
            echo "ROS 2 Jazzy is available. Run: ros2 topic list"
            docker run --rm $DOCKER_OPTIONS $IMAGE_NAME bash
        else
            echo "Starting Isaac Sim GUI with ROS 2 Simulation Control support..."
            docker run --rm $DOCKER_OPTIONS $IMAGE_NAME ./isaac-sim.sh --/isaac/startup/ros_sim_control_extension=True
        fi
    fi
else
    echo "Attaching to running container..."
    docker exec -it $CONTAINER_NAME /bin/bash
fi

