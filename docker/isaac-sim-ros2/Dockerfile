# Isaac Sim 5.1.0 with ROS 2 Jazzy (Ubuntu 24.04)
# Based on: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html

FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Switch to root for package installation
USER root

# Avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Fix apt directories and install prerequisites
RUN mkdir -p /var/lib/apt/lists/partial && \
    apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy base + required packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-control-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set ROS 2 environment variables
ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Copy and set entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Default working directory
WORKDIR /isaac-sim

# Use our ROS-enabled entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["./isaac-sim.sh"]

