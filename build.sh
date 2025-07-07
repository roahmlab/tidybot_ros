#!/usr/bin/env bash

# by @sethgi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
IMAGE_TAG=tidybot_platform

DOCKER_OPTIONS=""
DOCKER_OPTIONS+="-t $IMAGE_TAG:latest "
DOCKER_OPTIONS+="-f $SCRIPT_DIR/container.Dockerfile "
DOCKER_OPTIONS+="--build-arg USER_ID=$(id -u) --build-arg USER_NAME=$(whoami) "

DOCKER_CMD="docker build $DOCKER_OPTIONS $SCRIPT_DIR"
echo $DOCKER_CMD
exec $DOCKER_CMD