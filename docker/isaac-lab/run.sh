#!/bin/bash
set -e

# Helper function to print usage
usage() {
    echo "Usage: $0 [MODE] [COMMAND ...]"
    echo "Modes:"
    echo "  restart   : Stop existing container and start a new one (default)"
    echo "  start     : Start a new container if not running, or attach if running"
    echo "  ssh       : Headless mode for SSH (no X11/display forwarding)"
    echo "  shell     : Start container with interactive bash shell"
    echo ""
    echo "Examples:"
    echo "  $0 restart python.sh scripts/random_agent.py"
    echo "  $0 ssh python.sh scripts/rsl_rl/train.py --headless"
    echo "  $0 shell"
    echo "  $0 restart bash"
}

# Parse mode
MODE="restart"
SSH_MODE=false

case "$1" in
    restart|start)
        MODE="$1"
        shift
        ;;
    ssh)
        MODE="restart"
        SSH_MODE=true
        shift
        ;;
    shell)
        MODE="restart"
        shift
        set -- bash  # Set command to bash
        ;;
    -h|--help)
        usage
        exit 0
        ;;
esac

# Container name
CONTAINER_NAME="isaac-lab-tidybot"
IMAGE_NAME="isaac-lab-tidybot"

# Directories to map
TIDYBOT_ROOT="$(dirname "$(dirname "$(dirname "$(readlink -f "$0")")")")"

if [ "$MODE" == "restart" ]; then
    echo "Stopping existing container..."
    docker stop $CONTAINER_NAME 2>/dev/null || true
    docker rm $CONTAINER_NAME 2>/dev/null || true
fi

echo "Starting Isaac Lab TidyBot Container..."

# Build docker run command
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
    -w /workspace/tidybot_isaac
    -e ROS_DOMAIN_ID=0
    -v $TIDYBOT_ROOT/isaaclab/tidybot_isaac/:/workspace/tidybot_isaac
    -v $TIDYBOT_ROOT/src:/workspace/src
    -v $TIDYBOT_ROOT/docker/fastdds.xml:/fastdds.xml
    -v $HOME/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit
    -v $HOME/docker/isaac-sim/cache/ov:/root/.cache/ov
    -v $HOME/docker/isaac-sim/cache/pip:/root/.cache/pip
    -v $HOME/docker/isaac-sim/cache/gl:/root/.cache/nvidia/GLCache
    -v $HOME/docker/isaac-sim/cache/compute:/root/.nv/ComputeCache
    -v $HOME/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs
    -v $HOME/docker/isaac-sim/config:/root/.local/share/ov/data
    -v $HOME/docker/isaac-sim/documents:/root/Documents
)

# Add X11 forwarding only if not in SSH mode and DISPLAY is set
if [ "$SSH_MODE" = false ] && [ -n "$DISPLAY" ]; then
    echo "Enabling X11 forwarding..."
    xhost +local:root 1>/dev/null 2>&1 || true
    DOCKER_ARGS+=(
        -e DISPLAY=$DISPLAY
        -v /tmp/.X11-unix:/tmp/.X11-unix
    )
else
    echo "Running in headless mode (no display)..."
fi

docker run "${DOCKER_ARGS[@]}" $IMAGE_NAME "$@"

