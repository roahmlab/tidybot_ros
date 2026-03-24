## Finetuning Infrastructure
To register the RLDS dataset for training on, we provide an example `configs.py` and `transforms.py` to integrate with the openvla source code. These scripts should be placed [here](https://github.com/openvla/openvla/blob/main/prismatic/vla/datasets/rlds/oxe/configs.py#L54) and [here](https://github.com/openvla/openvla/blob/main/prismatic/vla/datasets/rlds/oxe/transforms.py#L828), respectively. 

## Deploying OpenVLA on Hardware

OpenVLA inference runs on a **remote GPU server** that communicates with the robot's onboard computer over the network using **CycloneDDS**. Both machines run the same TidyBot Docker container.

### Architecture

```
┌─────────────────────────────┐         CycloneDDS          ┌──────────────────────────────┐
│     Robot (Onboard PC)      │◄──────────────────────────► │      Remote GPU Server       │
│                             │                              │                              │
│  Hardware drivers           │   /camera_ext/compressed ──►│  OpenVLANode                 │
│  state_controller           │◄── /arm/delta_commands ──── │  (inference @ 5Hz)           │
│  moveit_ee_pose_ik          │                              │  (publish  @ 15Hz)           │
│  phone_teleop_server (:5000)│                              │                              │
└─────────────────────────────┘                              └──────────────────────────────┘
```

### Step 1: Build the Docker Image (Both Machines)

```bash
./docker/tidybot/build.sh
```

The Docker image includes CycloneDDS and Miniforge (conda). The OpenVLA conda environment is created at runtime (see Step 3).

### Step 2: CycloneDDS Configuration

CycloneDDS is required for cross-machine communication (FastDDS fragments images, causing corruption). Two config templates are provided in `docker/`:

- **`cyclonedds_local.xml`** — For the robot side. Fill in the robot's IP and the GPU server's IP.
- **`cyclonedds_server.xml`** — For the GPU server side. Uses auto-determined interface.

Pass `dds=cyclone` to `run.sh` and point to the config file:

```bash
# Robot
CYCLONEDDS_XML=docker/cyclonedds_local.xml ROS_DOMAIN_ID=1 \
    ./docker/tidybot/run.sh restart dds=cyclone

# GPU Server
CYCLONEDDS_XML=docker/cyclonedds_server.xml ROS_DOMAIN_ID=1 \
    ./docker/tidybot/run.sh restart dds=cyclone
```

### Step 3: GPU Server — Create OpenVLA Environment

Inside the container on the GPU server (first time only):

```bash
conda env create -f src/external/openvla/environment.yaml
conda activate openvla-deploy
```

### Step 4: Configure the VLA Node

Edit `src/tidybot_policy/tidybot_policy/remote_policy_vla.py`:

1. **PEFT adapter path** (line 72): Set to your fine-tuned LoRA checkpoint
2. **Dataset statistics path** (line 76): Set to your `dataset_statistics.json`
3. **Task prompt** (line 101): Update the instruction string
4. **Camera topic** (line 42): Choose `camera_ext` or `camera_wrist`

### Step 5: Launch

**Robot side** (two terminals inside the container):
```bash
# Terminal 1: Hardware drivers
ros2 launch tidybot_driver launch_hardware_robot.launch.py mode:=full ext_camera:=true

# Terminal 2: Supporting nodes (state controller, IK solver, WebXR UI)
ros2 launch tidybot_policy launch_remote_policy_vla.launch.py sim_mode:=hardware
```

**GPU server** (inside the container):
```bash
conda activate openvla-deploy
ros2 run tidybot_policy remote_policy_vla
```

Control inference via the WebXR UI at `http://<robot-ip>:5000`.

### OpenVLA Inference Details

The `OpenVLANode` loads the base OpenVLA model (`openvla/openvla-7b`) plus fine-tuned LoRA parameters via PEFT, and unnormalization statistics from the finetuning dataset.

- Subscribes to `/tidybot/camera_ext/color/compressed` (`sensor_msgs/CompressedImage`)
- Runs inference on the most recent image every **5 Hz**
- Publishes the latest action to `/tidybot/arm/delta_commands` (`std_msgs/Float64MultiArray`) at **15 Hz**
- Action format: 7-dim delta — Δ(x, y, z) position + Δ(roll, pitch, yaw) orientation + gripper state