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
    if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
        echo "Building Isaac Sim + ROS 2 image..."
        $SCRIPT_DIR/build.sh
        if [ $? -ne 0 ]; then exit 1; fi
    fi
}

# =============================================================================
# Setup directories and permissions
# =============================================================================
setup_directories() {
    mkdir -p $CACHE_DIR/cache/{kit,ov,pip,glcache,computecache}
    mkdir -p $CACHE_DIR/{config,data,logs,pkg,documents}
    
    ISAAC_WORKSPACE=$REPO_ROOT/isaac_sim_workspace
    mkdir -p $ISAAC_WORKSPACE

    # Ensure permissions (suppress errors if not owner)
    chmod -R 777 $CACHE_DIR $ISAAC_WORKSPACE 2>/dev/null || true
}

# =============================================================================
# Build Docker options
# =============================================================================
build_docker_options() {
    local RUN_MODE=$1

    # Resolve host group IDs
    RENDER_GID=$(getent group render 2>/dev/null | cut -d: -f3 || true)
    VIDEO_GID=$(getent group video 2>/dev/null | cut -d: -f3 || true)

    local OPS=""
    OPS+="--name $CONTAINER_NAME "
    OPS+="-it "
    OPS+="--privileged "   
    OPS+="--gpus all "
    OPS+="--network=host " # Host network required for both ROS 2 DDS and WebRTC
    
    # IPC Host prevents shared memory crashes in Isaac Sim
    OPS+="--ipc=host "

    # Resource limits
    OPS+="--ulimit nofile=65535:65535 "
    OPS+="--ulimit memlock=-1 "
    OPS+="--ulimit stack=67108864 "
    OPS+="--shm-size=16g "

    # Environment
    OPS+="-e ACCEPT_EULA=Y "
    OPS+="-e PRIVACY_CONSENT=Y "
    OPS+="-e OMNI_KIT_ALLOW_ROOT=1 "
    
    # Must include 'video' for Headless (NVENC) and 'display' for GUI
    OPS+="-e NVIDIA_VISIBLE_DEVICES=all "
    OPS+="-e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility,graphics,display "
    OPS+="-e __GLX_VENDOR_LIBRARY_NAME=nvidia "

    # Mounts
    OPS+="--device /dev/dri:/dev/dri "
    
    # Group Adds
    if [ -n "$VIDEO_GID" ]; then OPS+="--group-add $VIDEO_GID "; fi
    if [ -n "$RENDER_GID" ]; then OPS+="--group-add $RENDER_GID "; fi

    # --- Mode Specific Config ---
    if [ "$RUN_MODE" != "headless" ]; then
        # GUI/Shell Mode: Enable X11 Forwarding
        xhost +local: > /dev/null 2>&1
        OPS+="-e DISPLAY=$DISPLAY "
        OPS+="-v /tmp/.X11-unix:/tmp/.X11-unix:rw "
        OPS+="-v $HOME/.Xauthority:/root/.Xauthority:rw "
    fi
    # ----------------------------

    # Vulkan/EGL 
    if [ -d "/usr/share/vulkan/icd.d" ]; then
        OPS+="-v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro "
        OPS+="-e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json "
    fi
    if [ -d "/usr/share/glvnd/egl_vendor.d" ]; then
        OPS+="-v /usr/share/glvnd/egl_vendor.d:/usr/share/glvnd/egl_vendor.d:ro "
        OPS+="-e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json "
    fi

    # Isaac Sim Cache & Workspace
    OPS+="-v $CACHE_DIR/cache/kit:/isaac-sim/kit/cache/Kit:rw "
    OPS+="-v $CACHE_DIR/cache/ov:/root/.cache/ov:rw "
    OPS+="-v $CACHE_DIR/cache/pip:/root/.cache/pip:rw "
    OPS+="-v $CACHE_DIR/cache/glcache:/root/.cache/nvidia/GLCache:rw "
    OPS+="-v $CACHE_DIR/cache/computecache:/root/.nv/ComputeCache:rw "
    OPS+="-v $CACHE_DIR/logs:/root/.nvidia-omniverse/logs:rw "
    OPS+="-v $CACHE_DIR/config:/root/.nvidia-omniverse/config:rw "
    OPS+="-v $CACHE_DIR/data:/root/.local/share/ov/data:rw "
    OPS+="-v $ISAAC_WORKSPACE:/isaac_workspace:rw "
    OPS+="-v $CACHE_DIR/documents:/root/Documents:rw "

    # TidyBot Workspace
    OPS+="-v $REPO_ROOT/src/tidybot_description:/tidybot_description:rw "

    # ROS 2 DDS
    OPS+="-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} "
    OPS+="-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp "
    OPS+="-e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml "
    OPS+="-v $DOCKER_DIR/fastdds.xml:/fastdds.xml:ro "

    echo "$OPS"
}

# =============================================================================
# Run Container Logic
# =============================================================================
run_container() {
    local RUN_MODE=$1
    local OPTS=$(build_docker_options $RUN_MODE)

    if [ "$RUN_MODE" == "shell" ]; then
        echo "Starting container in SHELL mode..."
        echo "ROS 2 Jazzy is available. Run: ros2 topic list"
        docker run --rm $OPTS $IMAGE_NAME bash

    elif [ "$RUN_MODE" == "headless" ]; then
        echo "Fetching Public IP for WebRTC..."
        local PUB_IP=$(curl -s ifconfig.me)
        
        echo "Starting container in HEADLESS mode (WebRTC)..."
        echo "--------------------------------------------------------"
        echo "Public IP: $PUB_IP"
        echo "--------------------------------------------------------"

        docker run --rm \
            $OPTS \
            --runtime=nvidia \
            -e LIVESTREAM=2 \
            $IMAGE_NAME \
            ./runheadless.sh \
            --allow-root \
            --no-window \
            --ext:omni.services.streamclient.webrtc \
            --/app/window/width=1920 \
            --/app/window/height=1080 \
            --/exts/omni.kit.livestream.webrtc/public_ip=$PUB_IP \
            --/exts/omni.kit.livestream.webrtc/signaling_port=49100 \
            --/exts/omni.kit.livestream.webrtc/udp_port=47998
    else
        # DEFAULT: GUI Mode
        echo "Starting Isaac Sim GUI with ROS 2 support..."
        docker run --rm $OPTS $IMAGE_NAME
    fi
}

# =============================================================================
# Main Entry Point
# =============================================================================
print_usage() {
    echo "Usage: $0 [command] [mode]"
    echo ""
    echo "Commands:"
    echo "  (none)    Start/Attach container (Default: gui)"
    echo "  restart   Force restart container"
    echo "  build     Rebuild Docker image"
    echo ""
    echo "Modes:"
    echo "  gui       Run Isaac Sim Desktop App (Default)"
    echo "  headless  Run Isaac Sim with WebRTC streaming"
    echo "  shell     Run bash shell only (no sim)"
    echo ""
    echo "Examples:"
    echo "  $0                  # Start in GUI mode"
    echo "  $0 restart headless # Restart in Headless mode"
    echo "  $0 run shell        # Run bash shell"
}

# Arguments
COMMAND="${1:-run}"
MODE="${2:-gui}" # Default to GUI

# Handle Help
if [[ "$COMMAND" =~ ^(-h|--help)$ ]]; then
    print_usage
    exit 0
fi

# Handle Build
if [ "$COMMAND" == "build" ]; then
    echo "Force rebuilding Isaac Sim + ROS 2 image..."
    docker rmi $IMAGE_NAME 2>/dev/null || true
    $SCRIPT_DIR/build.sh
    exit $?
fi

# Standard Setup
build_image
setup_directories

# Execution Logic
if [ "$COMMAND" == "restart" ]; then
    echo "Force removing container..."
    docker rm -f $CONTAINER_NAME 2>/dev/null
    run_container $MODE

elif [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    # Container is not running
    if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        echo "Container exists but stopped. Restarting..."
        docker start -ai $CONTAINER_NAME
    else
        echo "Container does not exist. Creating new..."
        run_container $MODE
    fi
else
    # Container is running
    echo "Attaching to running container ($CONTAINER_NAME)..."
    docker exec -it $CONTAINER_NAME /bin/bash
fi