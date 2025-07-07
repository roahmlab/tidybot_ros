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
      python3-pip sudo vim wget \
      curl software-properties-common \
      doxygen git tmux \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get -y update \
    && apt-get -y install \
        libglew-dev libassimp-dev libboost-all-dev \
        libgtk-3-dev libglfw3-dev libavdevice-dev \
        libavcodec-dev libeigen3-dev libxxf86vm-dev \
        libembree-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get -y update \
    && apt-get -y install \ 
        cmake \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    # find the username for TARGET_UID (empty if none)
    USERNAME="$(getent passwd "${USER_ID}" | cut -d: -f1)"; \
    if [ -n "$USERNAME" ]; then \
      # delete user and their home directory
      userdel -r "$USERNAME"; \
    fi

# Create a new user with the specified USER_ID and USER_NAME
RUN useradd -m -l -u ${USER_ID} -s /bin/bash ${USER_NAME} \
    && usermod -aG video ${USER_NAME} \
    && export PATH=$PATH:/home/${USER_NAME}/.local/bin

RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/tidybot_platform
 
# Setup ROS 2 Jazzy + ROS 2 Control
RUN sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get install software-properties-common -y && \
    sudo apt-add-repository universe 

RUN sudo apt-get update && sudo apt-get install curl -y && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \ 
    /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \ 
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
    /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \ 
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \ 
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt-get update && sudo apt-get upgrade -y && \
    sudo apt-get install -y \
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
        ros-${ROS_DISTRO}-rqt-joint-trajectory-controller && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# Setup Gazebo
RUN sudo apt-get update && sudo apt-get upgrade && \
    sudo apt-get install ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control -y

# Setup MoveIt2 
RUN sudo apt-get install ros-${ROS_DISTRO}-moveit -y

# Setup Teleop
RUN sudo apt-get install python3-flask python3-flask-socketio -y

# Build the workspace
COPY ./src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && sudo rosdep init && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Setup base Tidybot packages
RUN sudo git clone https://github.com/jimmyyhwu/tidybot2.git /opt/tidybot2
RUN sudo curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-$(uname -m).sh && \
    sudo bash Miniforge3-Linux-$(uname -m).sh -b -p /opt/conda && sudo rm Miniforge3-Linux-$(uname -m).sh
RUN export PATH="/opt/conda/bin:$PATH" && \
    sudo chown -R $(whoami) /opt/conda && \
    conda install mamba -n base -c conda-forge && \
    mamba create -n tidybot2 python=3.10.14 -y && \
    mamba run -n tidybot2 pip install -r /opt/tidybot2/requirements.txt

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN sudo chown -R ${USER_NAME} /home/${USER_NAME}

COPY ./entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]