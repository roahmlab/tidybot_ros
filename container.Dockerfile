ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

ARG USER_NAME=default
ARG USER_ID=1000
ARG ROS_DISTRO=jazzy

# Prevent anything requiring user input
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=linux

ENV TZ=America
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Basic packages
RUN apt-get -y update \
    && apt-get -y install \
      cmake python3-pip python3.12-venv sudo vim wget \
      curl software-properties-common \
      doxygen git tmux dialog \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get -y update \
    && apt-get -y install \
        libglew-dev libassimp-dev libboost-all-dev \
        libgtk-3-dev libglfw3-dev libavdevice-dev \
        libavcodec-dev libeigen3-dev libxxf86vm-dev \
        libembree-dev iputils-ping usbutils can-utils \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    # find the username for TARGET_UID (empty if none)
    USERNAME="$(getent passwd "${USER_ID}" | cut -d: -f1)"; \
    if [ -n "$USERNAME" ]; then \
      # delete user and their home directory
      userdel -r "$USERNAME"; \
    fi

# Avoid shipping driver packages in the image
RUN apt-get purge -y 'nvidia-*' 'libnvidia-*' || true

# Create a new user with the specified USER_ID and USER_NAME
RUN useradd -m -l -u ${USER_ID} -s /bin/bash ${USER_NAME} \
    && usermod -aG video ${USER_NAME} \
    && export PATH=$PATH:/home/${USER_NAME}/.local/bin

RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Setup ROS 2 Jazzy + ROS 2 Control
RUN apt-get update && sudo apt-get upgrade -y && sudo apt-get install software-properties-common -y && \
    apt-add-repository universe 

RUN apt-get update && sudo apt-get install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \ 
    /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \ 
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
    /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \ 
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \ 
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Additional ros packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-rqt-controller-manager \
        ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
        ros-${ROS_DISTRO}-pinocchio && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# Setup Gazebo
RUN apt-get update && sudo apt-get upgrade && \
    apt-get install ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control -y

# Setup Gazebo sensors
RUN apt-get update && \
    apt-get install -y lsb-release wget gnupg && \
    echo "deb [arch=$(dpkg --print-architecture)] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable \
      $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-cache search libgz-sensors

# Setup MoveIt2 
RUN apt-get install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-visual-tools ros-${ROS_DISTRO}-moveit-servo -y

# Setup Teleop
RUN apt-get install python3-flask python3-flask-socketio -y

# Setup Phoenix6 and CANivore list for base control
RUN curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg" && \
    curl -s --compressed -o /etc/apt/sources.list.d/ctr2025.list "https://deb.ctr-electronics.com/ctr2025.list"
# Note: sudo apt install canivore-usb is requried for CANivore support, 
# but it is not available at build time. Do this manually after running the container.

# Build the workspace
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/tidybot_platform

COPY ./src ./src
COPY ./requirements.txt ./requirements.txt

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && sudo rosdep init && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN sudo chown -R ${USER_NAME} /home/${USER_NAME}

COPY ./entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]