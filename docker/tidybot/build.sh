#!/usr/bin/env bash
#
# Build the TidyBot platform Docker image
#

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_ROOT=$SCRIPT_DIR/../..
IMAGE_TAG=tidybot_platform

DOCKER_OPTIONS=""
DOCKER_OPTIONS+="-t $IMAGE_TAG:latest "
DOCKER_OPTIONS+="-f $SCRIPT_DIR/Dockerfile "
DOCKER_OPTIONS+="--build-arg USER_ID=$(id -u) --build-arg USER_NAME=$(whoami) "
DOCKER_OPTIONS+="--build-arg ROS_DISTRO=${ROS_DISTRO:-jazzy} "

DOCKER_CMD="docker build $DOCKER_OPTIONS $REPO_ROOT"
echo $DOCKER_CMD
exec $DOCKER_CMD

