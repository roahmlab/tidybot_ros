#!/usr/bin/env bash
#
# Build the Isaac Sim + ROS 2 Docker image
#

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
IMAGE_NAME=isaac-sim-ros2:5.1.0

echo "Building Isaac Sim + ROS 2 image..."
echo "This requires the base image. If not present, run:"
echo "  docker pull nvcr.io/nvidia/isaac-sim:5.1.0"
echo ""

docker build -t $IMAGE_NAME -f $SCRIPT_DIR/Dockerfile $SCRIPT_DIR

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Image built successfully: $IMAGE_NAME"
else
    echo ""
    echo "❌ Build failed. Make sure you have pulled the base image:"
    echo "  docker pull nvcr.io/nvidia/isaac-sim:5.1.0"
    exit 1
fi

