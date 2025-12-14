# Docker Configuration for TidyBot++ Platform

This directory contains Docker configuration files for running the TidyBot++ ROS 2 platform and NVIDIA Isaac Sim simulator.

## 📁 Directory Structure

```
docker/
├── build.sh              # Build the TidyBot platform Docker image
├── run.sh                # Run the TidyBot platform container
├── run_isaac_sim.sh      # Run the Isaac Sim container
├── container.Dockerfile  # Dockerfile for TidyBot platform
├── entrypoint.sh         # Container entrypoint script
├── fastdds_isaac.xml     # DDS configuration for ROS 2 communication
└── README.md             # This file
```

---

## 🚀 Quick Start

### 1. Build the TidyBot Platform Container

```bash
./docker/build.sh
```

### 2. Run the TidyBot Platform Container

```bash
./docker/run.sh restart
```

### 3. Run Isaac Sim Container (Optional)

```bash
./docker/run_isaac_sim.sh restart
```

---

## 📜 Script Reference

### `build.sh`

Builds the TidyBot platform Docker image with ROS 2 Jazzy and all dependencies.

```bash
./docker/build.sh
```

### `run.sh`

Runs the TidyBot platform container with full hardware access (CAN bus, cameras, GPU).

| Command | Description |
|---------|-------------|
| `./docker/run.sh restart` | Stop existing container and start fresh |
| `./docker/run.sh` | Start or attach to existing container |

### `run_isaac_sim.sh`

Runs NVIDIA Isaac Sim 5.1.0 for robot simulation.

| Command | Description |
|---------|-------------|
| `./docker/run_isaac_sim.sh restart` | Start the container with Isaac Sim GUI |
| `./docker/run_isaac_sim.sh restart shell` | Start container with bash shell |

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
3. **Docker image** pulled:
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

## 🔄 Converting URDF to USD for Isaac Sim

The TidyBot robot model needs to be converted from URDF to USD format for use in Isaac Sim.

### Step 1: Prepare the URDF

The Isaac Sim-compatible URDF is located at:
```
src/tidybot_description/urdf/tidybot_isaac.urdf
```

**Important Notes:**
- This URDF uses STL meshes instead of DAE (Isaac Sim doesn't fully support DAE)
- All mesh paths use the container mount point `/tidybot_description/...`
- The `scale="0.001 0.001 0.001"` attribute is required for gripper STL meshes (originally in mm)

### Step 2: Start Isaac Sim

```bash
./docker/run_isaac_sim.sh restart
```

Wait for the GUI to fully load (you'll see "Isaac Sim ... is loaded" in the terminal).

### Step 3: Import URDF

Refer to [Isaac Sim Tutorial: Import URDF](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html#isaac-sim-app-tutorial-advanced-import-urdf) for how to import the urdf model into the simulation.

**Important Notes**
- The `tidybot_isaac.urdf` is located at /tidybot_description/urdf/tidybot_isaac.urdf in the container.
- Select the ouput path (USD Ouput) as a writable one such as /tmp/ to avoid writing permission issue inside the container.

### Step 4: Verify the Import

After import, you should see the TidyBot robot in the viewport:
- Mobile base with Kinova mount
- Kinova Gen3 7-DOF arm
- Robotiq 2F-85 gripper

### Common Import Issues

| Error | Solution |
|-------|----------|
| `Failed to resolve mesh` | Ensure mesh paths start with `file:///tidybot_description/...`, which is as default in tidybot_isaac.urdf|
| `Used null prim` | Links have empty geometry tags - add valid geometry or remove, make sure no mesh file is missing |
| `No mass specified for link world` | Normal warning - the world link is a virtual anchor |

---

## 🔗 ROS 2 Communication Between Containers

Both containers are configured to communicate via ROS 2 DDS:

| Setting | Value |
|---------|-------|
| Network | `--network=host` |
| ROS_DOMAIN_ID | `0` (configurable via environment) |
| DDS Config | `fastdds_isaac.xml` |

### Verify Communication

In the **TidyBot container**:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

You should see topics from Isaac Sim when it's running with the ROS 2 bridge enabled.

---

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

1. Ensure both containers use the same `ROS_DOMAIN_ID`
2. Check that `--network=host` is set for both containers
3. Verify FastDDS configuration is mounted correctly

---

## 📚 Additional Resources

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Isaac Sim ROS 2 Bridge](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [TidyBot++ Project](https://tidybot2.github.io/)

