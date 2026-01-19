ARG BASE_IMAGE=isaac-lab-base
FROM ${BASE_IMAGE}

# Start as root
USER root

# Avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install curl and software-properties-common to set up ROS repositories
RUN apt-get update && apt-get install -y \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Base and Dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-control-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-msgs \
    ros-jazzy-action-msgs \
    ros-jazzy-rmw-fastrtps-cpp \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set ROS 2 environment variables
ENV ROS_DISTRO=jazzy


# Copy and set entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set working directory to the Isaac Lab root
WORKDIR ${ISAACLAB_PATH}

# Entrypoint setup
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
