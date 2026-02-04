#!/bin/bash
set -e

# Get the directory of the script
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
cd "$SCRIPT_DIR"

echo "Building Unified Isaac (Sim + Lab + ROS 2) Image..."
# Allow overriding BASE_IMAGE if needed, default to isaac-lab-base
BASE_IMAGE=${1:-isaac-lab-base}
echo "Using Base Image: $BASE_IMAGE"

docker build --build-arg BASE_IMAGE=$BASE_IMAGE -t isaac-tidybot .
