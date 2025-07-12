#!/usr/bin/env bash

# https://www.systec-electronic.com/en/demo/blog/article/news-socketcan-docker-the-solution
# run this script to set up a CAN interface in the Docker container

DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' tidybot_platform)
sudo ip link add vxcan0 type vxcan peer name vxcan1 netns $DOCKERPID
sudo modprobe can-gw
sudo cangw -A -s can0 -d vxcan0 -e
sudo cangw -A -s vxcan0 -d can0 -e
sudo ip link set vxcan0 up
sudo ip link set can0 type can bitrate 125000
sudo ip link set can0 up
sudo nsenter -t $DOCKERPID -n ip link set vxcan1 up