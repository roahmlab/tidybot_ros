ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

ARG USER_NAME=default
ARG USER_ID=1000

# Prevent anything requiring user input
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=linux

ENV TZ=America
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Basic packages
RUN apt-get -y update \
    && apt-get -y install \
      python3-pip \
      sudo \
      vim \
      wget \
      curl \
      software-properties-common \
      doxygen \
      git \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get -y update \
    && apt-get -y install \
        libglew-dev \
        libassimp-dev \
        libboost-all-dev \
        libgtk-3-dev \
        libglfw3-dev \
        libavdevice-dev \
        libavcodec-dev \
        libeigen3-dev \
        libxxf86vm-dev \
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
WORKDIR /home/${USER_NAME}/tidybot_ws
 
# Setup ROS 2 Jazzy
RUN sudo apt-get update && sudo apt-get upgrade && sudo apt-get install software-properties-common -y && \
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

RUN sudo apt-get update && sudo apt-get install ros-dev-tools -y

RUN sudo apt-get update && sudo apt-get upgrade && \
    sudo apt-get install ros-jazzy-desktop -y

RUN sudo apt-get install python3-colcon-common-extensions \
    python3-rosdep -y

# Setup Gazebo
RUN sudo apt-get update && sudo apt-get upgrade && sudo apt-get install ros-jazzy-ros-gz -y

# Setup MoveIt2 
RUN sudo apt-get install ros-jazzy-moveit -y

# Build the workspace
COPY ./src ./src
RUN . /opt/ros/jazzy/setup.sh && sudo rosdep init && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

RUN sudo chown -R ${USER_NAME} /home/${USER_NAME}

COPY ./entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]