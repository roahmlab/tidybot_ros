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
# Basic utilities and development tools
RUN apt-get -y update \
    && apt-get -y install \
      apt-transport-https \
      ca-certificates \
      gnupg \
      lsb-release \
      udev \
      unzip \
      zip \
      cmake python3-pip python3-venv sudo vim wget \
      curl software-properties-common \
      doxygen git tmux dialog dialog \
      && rm -rf /var/lib/apt/lists/*

# Graphics and robotics dependencies
RUN apt-get -y update \
    && apt-get -y install \
        libglew-dev libassimp-dev libboost-all-dev \
        libgtk-3-dev libglfw3-dev libavdevice-dev \
        libavcodec-dev libeigen3-dev libxxf86vm-dev \
        libembree-dev iputils-ping usbutils can-utils \
        libgl1-mesa-dri libegl1 libgbm1 mesa-vulkan-drivers mesa-utils vulkan-tools \
        gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ROS and Orbbec SDK runtime dependencies
RUN apt-get -y update \
    && apt-get -y install \
        libgflags-dev nlohmann-json3-dev libdw-dev \
        libssl-dev libusb-1.0-0-dev libudev-dev \
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
RUN apt-get update && apt-get install -y software-properties-common && \
    apt-add-repository universe 

RUN apt-get update && apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
    /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Additional ros packages
RUN apt-get update && apt-get install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-rqt-controller-manager \
        ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
        ros-${ROS_DISTRO}-pinocchio \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-compressed-image-transport \
        ros-${ROS_DISTRO}-image-publisher \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-diagnostic-msgs \
        ros-${ROS_DISTRO}-statistics-msgs \
        ros-${ROS_DISTRO}-backward-ros \
        ros-${ROS_DISTRO}-simulation-interfaces && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Setup Gazebo
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control

# Setup Gazebo sensors
RUN apt-get update && \
    echo "deb [arch=$(dpkg --print-architecture)] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable \
      $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update

# Setup MoveIt2 
RUN apt-get install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-visual-tools ros-${ROS_DISTRO}-moveit-servo -y

# Setup Teleop
RUN apt-get install python3-flask python3-flask-socketio -y

# Episode converters: Apache Arrow/Parquet and HDF5 dev libs for tidybot_episode
RUN apt-get update && apt-get install -y ca-certificates lsb-release wget && \
        wget -q https://apache.jfrog.io/artifactory/arrow/ubuntu/apache-arrow-apt-source-latest-$(lsb_release -cs).deb && \
        apt-get install -y ./apache-arrow-apt-source-latest-$(lsb_release -cs).deb && \
        rm -f apache-arrow-apt-source-latest-$(lsb_release -cs).deb && \
        apt-get update && apt-get install -y \
            libarrow-dev \
            libparquet-dev \
            libhdf5-dev \
        && rm -rf /var/lib/apt/lists/*

# Setup Phoenix6 and CANivore for base control
RUN curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg" && \
    curl -s --compressed -o /etc/apt/sources.list.d/ctr2025.list "https://deb.ctr-electronics.com/ctr2025.list"
# Note: canivore-usb and canivore-usb-kernel are required for CANivore support, 
# but it is not available at build time. Do this manually after running the container.

# Build the workspace
WORKDIR /home/${USER_NAME}/tidybot_platform

COPY ./src ./src
COPY ./requirements.txt ./requirements.txt

# Fetch external dependencies via vcstool (as root during build)
RUN cd /home/${USER_NAME}/tidybot_platform/src/external && \
    vcs import . < orbbec.repos || true

# Install Orbbec udev rules (reload may fail in build env; ignore non-zero exit)
RUN bash /home/${USER_NAME}/tidybot_platform/src/external/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh || true

# Initialize rosdep as root, then switch to user for build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && (rosdep init || true)

# Ensure ownership for the workspace
RUN chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

USER ${USER_NAME}

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys \
    "arrow parquet warehouse_ros_mongo ament_python glog joint_state_publisher \
    joint_state_publisher_gui opencv opencv4 threadpoolctl setuptools numpy \
    libgoogle-glog-dev flask flask_socketio" && \
    colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc

COPY --chmod=0755 ./docker/tidybot/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

