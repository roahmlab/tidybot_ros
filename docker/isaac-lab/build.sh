#!/bin/bash
set -e

# Get the directory of the script
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
cd "$SCRIPT_DIR"

echo "Building Isaac Lab ROS 2 TidyBot Image..."
docker build -t isaac-lab-tidybot .
