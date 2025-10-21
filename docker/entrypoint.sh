#!/bin/bash

sudo ldconfig

if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

if [ -f "/home/${USER}/tidybot_platform/install/setup.bash" ]; then
  source "/home/${USER}/tidybot_platform/install/setup.bash"
fi

exec "/bin/bash"