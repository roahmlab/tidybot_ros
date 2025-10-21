#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
IMAGE_TAG=tidybot_platform
CONTAINER_NAME=tidybot_platform
DATA_DIR=$SCRIPT_DIR/../
XRUN="/run/user/$(id -u)"
USER_NAME=${USER_NAME:-$(whoami)}

# Resolve host group IDs for reliable permission mapping inside container
RENDER_GID=$(getent group render | cut -d: -f3 || true)
VIDEO_GID=$(getent group video | cut -d: -f3 || true)

capabilities_str=\""capabilities=compute,utility,graphics,display\""

DOCKER_OPTIONS=""
DOCKER_OPTIONS+="-it "
DOCKER_OPTIONS+="-e DISPLAY=$DISPLAY "
DOCKER_OPTIONS+="-v /tmp/.X11-unix:/tmp/.X11-unix "
DOCKER_OPTIONS+="-v $DATA_DIR:/home/$(whoami)/Documents/tidybot_platform "
DOCKER_OPTIONS+="-v /etc/group:/etc/group:ro "
DOCKER_OPTIONS+="-v /mnt/:/mnt/hostmnt "
# installing the canivore-usb package may modify the host modules
DOCKER_OPTIONS+="-v /lib/modules:/lib/modules:rw "
DOCKER_OPTIONS+="-v $XRUN:$XRUN:rw "
DOCKER_OPTIONS+="-v /dev/input:/dev/input:ro "
DOCKER_OPTIONS+="-e XDG_RUNTIME_DIR=$XRUN "
DOCKER_OPTIONS+="--name $CONTAINER_NAME "
DOCKER_OPTIONS+="--privileged "
DOCKER_OPTIONS+="--net=host "
DOCKER_OPTIONS+="-e SDL_VIDEODRIVER=x11 "
DOCKER_OPTIONS+="-u $(id -u):$(id -g) "
DOCKER_OPTIONS+="--shm-size 32G "
DOCKER_OPTIONS+="-m 32g "
DOCKER_OPTIONS+="--cap-add SYS_MODULE "
DOCKER_OPTIONS+="--device /dev/dri:/dev/dri "
if [ -n "$VIDEO_GID" ]; then DOCKER_OPTIONS+="--group-add $VIDEO_GID "; fi
if [ -n "$RENDER_GID" ]; then DOCKER_OPTIONS+="--group-add $RENDER_GID "; fi

XAUTH=/tmp/.docker.xauth
if [ ! -f "$XAUTH" ]; then
  touch "$XAUTH"
  xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
fi
chmod a+r "$XAUTH"
DOCKER_OPTIONS+="-e XAUTHORITY=$XAUTH "
DOCKER_OPTIONS+="-v $XAUTH:$XAUTH:ro "

for cam in /dev/video*; do
  DOCKER_OPTIONS+="--device=${cam} "
done

if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
  echo "✔ NVIDIA GPU detected. Enabling GPU support."
  DOCKER_OPTIONS+="--gpus all "
  DOCKER_OPTIONS+="-e NVIDIA_VISIBLE_DEVICES=all "
  DOCKER_OPTIONS+="-e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display "
  DOCKER_OPTIONS+="-e __GLX_VENDOR_LIBRARY_NAME=nvidia "
  # Mount host NVIDIA ICD/EGL vendor JSONs so Vulkan/EGL can find the driver in-container
  if [ -d "/usr/share/vulkan/icd.d" ]; then
    DOCKER_OPTIONS+="-v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro "
    # Hint Vulkan to NVIDIA ICD (path inside container after bind-mount)
    DOCKER_OPTIONS+="-e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json "
  fi
  if [ -d "/usr/share/glvnd/egl_vendor.d" ]; then
    DOCKER_OPTIONS+="-v /usr/share/glvnd/egl_vendor.d:/usr/share/glvnd/egl_vendor.d:ro "
    DOCKER_OPTIONS+="-e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json "
  fi
else
  echo "No NVIDIA GPU found. Running without GPU support."
  DOCKER_OPTIONS+="-e MESA_LOADER_DRIVER_OVERRIDE=${MESA_LOADER_DRIVER_OVERRIDE:-} "
  DOCKER_OPTIONS+="-e LIBGL_DRI3_DISABLE=${LIBGL_DRI3_DISABLE:-0} "
  DOCKER_OPTIONS+="-e LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-0} "
fi

echo $CONTAINER_NAME

if [ ${1:-""} == "restart" ]; then 
  echo "Restarting Container"
  docker rm -f $CONTAINER_NAME
  docker run $DOCKER_OPTIONS $IMAGE_TAG /bin/bash
# https://stackoverflow.com/questions/38576337/how-to-execute-a-bash-command-only-if-a-docker-container-with-a-given-name-does
elif [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then # If container isn't running
    
    # If it exists, but needs to be started
    if [  "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
          echo "Resuming Container"
          docker start $CONTAINER_NAME
          docker exec -it $CONTAINER_NAME /entrypoint.sh
    else
      echo "Running Container"
      docker run $DOCKER_OPTIONS $IMAGE_TAG:latest /entrypoint.sh
    fi
else
  echo "Attaching to existing container"
  docker exec -it $CONTAINER_NAME /entrypoint.sh
fi
