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
├── isaac-sim-ros2/           # Isaac Sim + ROS 2 container
│   ├── Dockerfile            # Isaac Sim 5.1.0 + ROS 2 Jazzy
│   ├── build.sh              # Build the Isaac Sim image
│   ├── run.sh                # Run the Isaac Sim container
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

### Isaac Sim with ROS 2

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

## 🔄 Converting URDF to USD for Isaac Sim

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
./docker/isaac-sim-ros2/run.sh restart
```

Wait for the GUI to fully load (you'll see "Isaac Sim ... is loaded" in the terminal).

### Step 3: Import URDF

Refer to [Isaac Sim Tutorial: Import URDF](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html) for detailed instructions.

**Important Notes:**
- The `tidybot_isaac.urdf` is at `/tidybot_description/urdf/tidybot_isaac.urdf` in the container
- Set output path (USD Output) to a writable location like `/tmp/` or `/isaac_workspace/`

### Step 4: Verify the Import

After import, you should see the TidyBot robot in the viewport:
- Mobile base with Kinova mount
- Kinova Gen3 7-DOF arm
- Robotiq 2F-85 gripper

### Common Import Issues

| Error | Solution |
|-------|----------|
| `Failed to resolve mesh` | Ensure mesh paths start with `file:///tidybot_description/...` |
| `Used null prim` | Links have empty geometry tags - add valid geometry or remove |
| `No mass specified for link world` | Normal warning - the world link is a virtual anchor |

---

## 🔗 ROS 2 Communication Between Containers

Both containers are configured to communicate via ROS 2 DDS using the shared `fastdds.xml` configuration.

| Setting | Value |
|---------|-------|
| Network | `--network=host` |
| ROS_DOMAIN_ID | `0` (configurable via environment) |
| RMW_IMPLEMENTATION | `rmw_fastrtps_cpp` |
| DDS Config | `docker/fastdds.xml` |

### Verify Communication

**In the TidyBot container:**
```bash
ros2 topic list
```

**In the Isaac Sim container (with ROS 2 bridge enabled):**
```bash
ros2 topic list
ros2 topic echo /clock
```

You should see topics from both containers when Isaac Sim is running with the ROS 2 bridge enabled.

### Setting Up the ROS 2 Bridge in Isaac Sim

After importing the robot, use the Script Editor (Window > Script Editor) to set up the ActionGraph:

```python
import omni.graph.core as og

og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ReadSimTime.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "/clock"),
            ("PublishJointState.inputs:topicName", "/joint_states"),
            ("SubscribeJointState.inputs:topicName", "/joint_command"),
            ("ArticulationController.inputs:robotPath", "/World/tidybot/joint_x_jointbody"),
        ],
    },
)
```

After running the script, set the `targetPrim` for `PublishJointState` to your robot's articulation root.

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
