#!/bin/bash
set -e

# Helper function to print usage
usage() {
    echo "Usage: $0 [MODE] [COMMAND ...]"
    echo "Modes:"
    echo "  restart   : Stop existing container and start a new one (default)"
    echo "  start     : Start a new container if not running, or attach if running"
    echo "  shell     : Start a new container and enter shell (no GUI automatic start)"
    echo ""
    echo "Examples:"
    echo "  $0 restart"
    echo "  $0 restart shell"
}

MODE=${1:-restart}
shift

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

# Enable X11 forwarding
xhost +local:root 1>/dev/null 2>&1

docker run -it --rm \
    --name $CONTAINER_NAME \
    --network=host \
    --privileged \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $TIDYBOT_ROOT/src/tidybot_description:/workspace/src/tidybot_description \
    -v $TIDYBOT_ROOT/docker/fastdds.xml:/fastdds.xml \
    -v $HOME/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit \
    -v $HOME/docker/isaac-sim/cache/ov:/root/.cache/ov \
    -v $HOME/docker/isaac-sim/cache/pip:/root/.cache/pip \
    -v $HOME/docker/isaac-sim/cache/gl:/root/.cache/nvidia/GLCache \
    -v $HOME/docker/isaac-sim/cache/compute:/root/.nv/ComputeCache \
    -v $HOME/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs \
    -v $HOME/docker/isaac-sim/config:/root/.local/share/ov/data \
    -v $HOME/docker/isaac-sim/documents:/root/Documents \
    $IMAGE_NAME \
    "$@"
