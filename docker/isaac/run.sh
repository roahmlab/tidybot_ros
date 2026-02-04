#!/bin/bash
set -e

# =============================================================================
# Usage
# =============================================================================
usage() {
    echo "Usage: $0 [MODE] [COMMAND ...]"
    echo "Modes:"
    echo "  sim       : Launch Isaac Sim directly (./isaac-sim.sh)"
    echo "  lab       : Start container in Isaac Lab workspace (interactive bash)"
    echo "  shell     : Alias for 'lab'"
    echo "  headless  : Run without X11 forwarding (useful for SSH/training)"
    echo ""
    echo "Examples:"
    echo "  $0 sim                        # Launch Isaac Sim GUI"
    echo "  $0 lab                        # Enter container shell to run Lab scripts"
    echo "  $0 headless python scripts/train.py  # Run training headlessly"
}

# =============================================================================
# Configuration
# =============================================================================
CONTAINER_NAME="isaac-tidybot"
IMAGE_NAME="isaac-tidybot"

# Directory setup
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
TIDYBOT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
CACHE_DIR="$HOME/docker/isaac-sim"

# =============================================================================
# Parse Arguments
# =============================================================================
MODE="lab"
CMD=""

if [ "$#" -ge 1 ]; then
    case "$1" in
        sim)
            MODE="sim"
            shift
            ;;
        lab|shell)
            MODE="lab"
            shift
            ;;
        headless)
            MODE="headless"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            # If unknown, assume it's a command for the default 'lab' mode? 
            # Or just default to lab mode.
            MODE="lab"
            ;;
    esac
fi

# =============================================================================
# Setup Cache Directories (Standard Isaac Sim pattern)
# =============================================================================
mkdir -p $CACHE_DIR/cache/{kit,ov,pip,glcache,computecache}
mkdir -p $CACHE_DIR/{config,data,logs,pkg,documents}

# =============================================================================
# Docker Arguments
# =============================================================================
DOCKER_ARGS=(
    -it --rm
    --name $CONTAINER_NAME
    --network=host
    --privileged
    --gpus all
    --ulimit nproc=-1
    --ulimit nofile=100000:100000
    --ulimit stack=67108864
    --ulimit memlock=-1
    -e ACCEPT_EULA=Y
    -e PRIVACY_CONSENT=Y
    -e OMNI_KIT_ALLOW_ROOT=1
    -e ROS_DOMAIN_ID=0
    # Mount TidyBot Sources
    -v $TIDYBOT_ROOT/isaaclab/tidybot_isaac/:/workspace/tidybot_isaac
    -v $TIDYBOT_ROOT/src:/workspace/src
    -v $TIDYBOT_ROOT/docker/fastdds.xml:/fastdds.xml
    # Mount Isaac Sim Cache/Data
    -v $CACHE_DIR/cache/kit:/isaac-sim/kit/cache/Kit
    -v $CACHE_DIR/cache/ov:/root/.cache/ov
    -v $CACHE_DIR/cache/pip:/root/.cache/pip
    -v $CACHE_DIR/cache/glcache:/root/.cache/nvidia/GLCache
    -v $CACHE_DIR/cache/computecache:/root/.nv/ComputeCache
    -v $CACHE_DIR/logs:/root/.nvidia-omniverse/logs
    -v $CACHE_DIR/config:/root/.local/share/ov/data
    -v $CACHE_DIR/documents:/root/Documents
)

# X11 / Display Logic
if [ "$MODE" != "headless" ] && [ -n "$DISPLAY" ]; then
    echo "Enabling X11 forwarding..."
    xhost +local:root 1>/dev/null 2>&1 || true
    DOCKER_ARGS+=(
        -e DISPLAY=$DISPLAY
        -v /tmp/.X11-unix:/tmp/.X11-unix
        -v $HOME/.Xauthority:/root/.Xauthority:rw
    )
else
    echo "Running in headless mode..."
fi

# =============================================================================
# Build Check
# =============================================================================
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    echo "Image $IMAGE_NAME not found. Building..."
    $SCRIPT_DIR/build.sh
fi

# =============================================================================
# Execution
# =============================================================================
CMD=("/bin/bash")

if [ "$MODE" == "sim" ]; then
    echo "Starting shell in Isaac Sim directory..."
    WORKDIR="/isaac-sim"
    # User can run ./isaac-sim.sh from here manually
else
    echo "Starting shell in Isaac Lab workspace..."
    # As requested: Enter the Isaac Lab project directory @[isaaclab/tidybot_isaac]
    WORKDIR="/workspace/tidybot_isaac"
fi

# If user provided extra args, pass them to bash (e.g. -c "python script.py")
# or override CMD if they know what they are doing
if [ "$#" -gt 0 ]; then
   CMD=("$@")
fi

# Run
docker run "${DOCKER_ARGS[@]}" -w "$WORKDIR" $IMAGE_NAME "${CMD[@]}"
