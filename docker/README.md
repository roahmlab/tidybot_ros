# Docker Configuration for TidyBot++ Platform

This directory contains Docker configurations for running the TidyBot++ ROS 2 platform and NVIDIA Isaac Sim simulator.

## 📁 Directory Structure

```
docker/
├── tidybot/                  # TidyBot platform container
│   ├── Dockerfile            # Ubuntu 24.04 + ROS 2 Jazzy + workspace
│   ├── build.sh              # Build the TidyBot image
│   ├── run.sh                # Run the TidyBot container
│   └── entrypoint.sh         # Container entrypoint script
│
├── isaac-sim-ros2/           # Isaac Sim + ROS 2 container (Standard Simulation)
│   ├── Dockerfile            # Isaac Sim 5.1.0 + ROS 2 Jazzy
│   ├── build.sh              # Build the Isaac Sim image
│   ├── run.sh                # Run the Isaac Sim container
│   └── entrypoint.sh         # Container entrypoint script
│
├── isaac-lab/                # Isaac Lab + ROS 2 container (RL Policy Training)
│   ├── Dockerfile            # Ubuntu 24.04 (Isaac Lab Base) + ROS 2 Jazzy
│   ├── build.sh              # Build the Isaac Lab image
│   ├── run.sh                # Run the Isaac Lab container
│   └── entrypoint.sh         # Container entrypoint script
│
├── fastdds.xml               # Shared DDS config for cross-container ROS 2
└── README.md                 # This file
```

---

## 🚀 Quick Start

### TidyBot Platform

```bash
# Build the image (first time only)
./docker/tidybot/build.sh

# Run the container
./docker/tidybot/run.sh restart
```


### Isaac Lab with ROS 2 (RL Training)

This container provides the Isaac Lab framework for training Reinforcement Learning policies.

**Prerequisite:** You must build the `isaac-lab-base` image from the official Isaac Lab repository first.

1.  **Clone and build Isaac Lab Base:**
    ```bash
    # Clone Isaac Lab
    git clone https://github.com/isaac-sim/IsaacLab.git
    cd IsaacLab
    
    # Build the base image
    ./docker/container.py build base
    ```

2.  **Build the TidyBot extension:**
    Return to the `tidybot_platform` directory:
    ```bash
    # Build the image (depends on isaac-lab-base)
    ./docker/isaac-lab/build.sh
    
    # Run the container
    ./docker/isaac-lab/run.sh restart
    ```

### Isaac Sim with ROS 2 (Standard Simulation)

This is the standard container for running TidyBot simulations with ROS 2 Bridge.

```bash
# Pull the base image (first time only)
docker pull nvcr.io/nvidia/isaac-sim:5.1.0

# Build and run (image is built automatically if not present)
./docker/isaac-sim-ros2/run.sh restart

# Or build manually first
./docker/isaac-sim-ros2/build.sh
./docker/isaac-sim-ros2/run.sh restart
```

---

## 📜 Script Reference

### TidyBot Container (`docker/tidybot/`)

| Script | Description |
|--------|-------------|
| `build.sh` | Build the TidyBot platform Docker image with ROS 2 Jazzy |
| `run.sh restart` | Stop existing container and start fresh |
| `run.sh` | Start or attach to existing container |


### Isaac Lab Container (`docker/isaac-lab/`)

| Script | Description |
|--------|-------------|
| `build.sh` | Build the Isaac Lab image (Ubuntu 24.04 + ROS 2 Jazzy) |
| `run.sh restart` | Stop existing container and start fresh |
| `run.sh` | Start or attach to existing container |
| `run.sh restart shell` | Start container and enter shell immediately |

### Isaac Sim Container (`docker/isaac-sim-ros2/`)

| Script | Description |
|--------|-------------|
| `build.sh` | Build the Isaac Sim + ROS 2 Jazzy image |
| `run.sh restart` | Start the container with Isaac Sim GUI |
| `run.sh restart shell` | Start container with bash shell only |
| `run.sh build` | Force rebuild the Docker image |

---

## 🎮 Isaac Sim Integration

### Prerequisites

1. **NVIDIA GPU** with driver version ≥ 535.x
2. **NVIDIA Container Toolkit** installed:
   ```bash
   sudo apt-get install nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   ```
3. **Base Docker image** pulled:
   ```bash
   docker pull nvcr.io/nvidia/isaac-sim:5.1.0
   ```

### Container Mounts

| Host Path | Container Path | Description |
|-----------|----------------|-------------|
| `src/tidybot_description/` | `/tidybot_description` | Robot URDF and meshes |
| `isaac_sim_workspace/` | `/isaac_workspace` | USD output directory |
| `~/docker/isaac-sim/` | Various cache paths | Persistent cache |

---

## 🔗 ROS 2 Communication Between Containers

Both containers are configured to communicate via ROS 2 DDS using the shared `fastdds.xml` configuration.

| Setting | Value |
|---------|-------|
| Network | `--network=host` |
| ROS_DOMAIN_ID | `0` (configurable via environment) |
| RMW_IMPLEMENTATION | `rmw_fastrtps_cpp` |
| DDS Config | `docker/fastdds.xml` |

## 🛠️ Troubleshooting

### Isaac Sim Won't Start

1. **Check GPU access:**
   ```bash
   docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
   ```

2. **Check X11 permissions:**
   ```bash
   xhost +local:
   ```

3. **On Wayland:** Switch to X11 session or set:
   ```bash
   export DISPLAY=:0
   ```

### Container Can't Write to Mounted Directories

```bash
# Fix permissions for Isaac Sim workspace
chmod -R 777 ~/Documents/tidybot_platform/isaac_sim_workspace/
chmod -R 777 ~/docker/isaac-sim/
```

### ROS 2 Topics Not Visible Between Containers

1. Ensure both containers use the same `ROS_DOMAIN_ID`:
   ```bash
   export ROS_DOMAIN_ID=0
   ```

2. Check that `--network=host` is set for both containers

3. Verify FastDDS configuration is mounted correctly:
   ```bash
   # Inside container
   cat /fastdds.xml
   echo $FASTRTPS_DEFAULT_PROFILES_FILE
   ```

4. Restart the ROS 2 daemon in both containers:
   ```bash
   ros2 daemon stop && ros2 daemon start
   ```

### Isaac Sim ROS 2 Bridge Not Publishing

1. Make sure the simulation is **playing** (not paused)
2. Check that `isaacsim.ros2.bridge` extension is enabled (Window > Extensions)
3. Verify the ActionGraph is properly configured
4. Check the Isaac Sim console for errors

---

## 📚 Additional Resources

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Isaac Sim ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Isaac Sim ROS 2 Tutorials](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html)
- [TidyBot++ Project](https://tidybot2.github.io/)
