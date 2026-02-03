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
├── isaac/                    # Isaac Sim + Isaac Lab + ROS 2 container
│   ├── Dockerfile            # Isaac Lab Base + ROS 2 Jazzy
│   ├── build.sh              # Build the Isaac image
│   ├── run.sh                # Run the Isaac container
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

### Isaac Sim + Isaac Lab (Simulation & RL Training)

This container combines Isaac Sim for simulation and Isaac Lab for RL training.

**Prerequisites:** Build the `isaac-lab-base` image from the official Isaac Lab repository first:
```bash
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./docker/container.py build base
```

Then build and run the TidyBot Isaac container:
```bash
# Build the image
./docker/isaac/build.sh

# Run with Isaac Sim GUI (starts in /isaac-sim)
./docker/isaac/run.sh sim

# Run in Isaac Lab workspace (starts in /workspace/tidybot_isaac)
./docker/isaac/run.sh lab

# Run headlessly (for training)
./docker/isaac/run.sh headless python scripts/train.py
```

---

## 📜 Script Reference

### TidyBot Container (`docker/tidybot/`)

| Script | Description |
|--------|-------------|
| `build.sh` | Build the TidyBot platform Docker image with ROS 2 Jazzy |
| `run.sh restart` | Stop existing container and start fresh |
| `run.sh` | Start or attach to existing container |

### Isaac Container (`docker/isaac/`)

| Script | Description |
|--------|-------------|
| `build.sh` | Build the image (requires `isaac-lab-base` as base) |
| `run.sh sim` | Start shell in `/isaac-sim` directory (run `./isaac-sim.sh` to launch GUI) |
| `run.sh lab` | Start shell in `/workspace/tidybot_isaac` (Isaac Lab workspace) |
| `run.sh shell` | Alias for `lab` |
| `run.sh headless [CMD]` | Run without X11 forwarding, optionally with a command |

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
3. **Isaac Lab Base Image** built from [IsaacLab repository](https://github.com/isaac-sim/IsaacLab)

### Container Mounts

| Host Path | Container Path | Description |
|-----------|----------------|-------------|
| `isaaclab/tidybot_isaac/` | `/workspace/tidybot_isaac` | Isaac Lab extensions |
| `src/` | `/workspace/src` | ROS 2 packages (URDF, meshes) |
| `docker/fastdds.xml` | `/fastdds.xml` | DDS configuration |
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
   xhost +local:root
   ```

3. **On Wayland:** Switch to X11 session or set:
   ```bash
   export DISPLAY=:0
   ```

### Container Can't Write to Mounted Directories

```bash
# Fix permissions
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
- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)
- [TidyBot++ Project](https://tidybot2.github.io/)
