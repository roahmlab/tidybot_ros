# Docker Configuration for TidyBot++ Platform

This directory contains Docker configurations for running the TidyBot++ ROS 2 platform and NVIDIA Isaac Sim simulator.

## 📁 Directory Structure

```
docker/
├── tidybot/                  # TidyBot platform container
│   ├── Dockerfile            # Ubuntu 24.04 + ROS 2 Jazzy + Miniforge + workspace
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
├── fastdds.xml               # DDS config for local cross-container ROS 2
├── cyclonedds.xml            # CycloneDDS config for cross-machine ROS 2
├── diagnose_dds.sh           # DDS connectivity diagnostic script
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
| `run.sh restart` | Stop existing container and start fresh (FastDDS) |
| `run.sh restart dds=cyclone` | Start fresh with CycloneDDS (for cross-machine VLA) |
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

## 🔗 ROS 2 Communication Between Containers

Both containers are configured to communicate via ROS 2 DDS using the shared `fastdds.xml` configuration.

| Setting | Value |
|---------|-------|
| Network | `--network=host` |
| ROS_DOMAIN_ID | `0` (configurable via environment) |
| RMW_IMPLEMENTATION | `rmw_fastrtps_cpp` |
| DDS Config | `docker/fastdds.xml` |

## 🌐 Cross-Machine Communication (CycloneDDS)

For communication between the robot and a remote GPU server (e.g., for VLA inference), use **CycloneDDS** instead of FastDDS. FastDDS fragments published images during cross-machine transport, leading to corrupted data.

### Configuration

Edit `docker/cyclonedds.xml` on each machine:

**Client side (robot / laptop):**
```xml
<NetworkInterface address="<CLIENT_IP>"/>   <!-- This machine's IP -->
<Peer address="<SERVER_IP>"/>               <!-- Remote machine's IP -->
```

**Server side (GPU server):**
```xml
<NetworkInterface address="<SERVER_IP>"/>   <!-- This machine's IP -->
<Peer address="<CLIENT_IP>"/>               <!-- Remote machine's IP -->
```

Both sides **must** use the same `Domain Id` and `MaxMessageSize`.

### Usage

Pass `dds=cyclone` to `run.sh`:

```bash
# Client side (robot / laptop)
./docker/tidybot/run.sh restart dds=cyclone

# Server side (GPU server) — outside Docker
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=$(realpath docker/cyclonedds.xml)
export ROS_DOMAIN_ID=0
```

If `CYCLONEDDS_XML` is not set, `run.sh` defaults to `docker/cyclonedds.xml`.

For full VLA deployment instructions, see [`src/external/openvla/README.md`](../src/external/openvla/README.md).

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

### CycloneDDS Cross-Machine Connection Not Working

Use the diagnostic script to check both machines:

```bash
bash docker/diagnose_dds.sh <PEER_IP>
```

#### 1. Verify UDP connectivity in both directions

DDS requires **bidirectional UDP**. Test with:

```bash
# On machine A (listen):
python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 7400))
s.settimeout(10)
print('Listening on UDP 7400...')
try:
    data, addr = s.recvfrom(1024)
    print(f'SUCCESS: Received from {addr}')
except socket.timeout:
    print('FAIL: No packet received')
"

# On machine B (send):
python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'hello', ('<MACHINE_A_IP>', 7400))
print('Sent')
"
```

Test **both directions** (swap A and B). If one direction fails, the firewall on the receiving machine is blocking inbound UDP.

#### 2. Open the local firewall

If `ufw` is active, it will block inbound UDP by default:

```bash
# Check firewall status
sudo ufw status

# Allow UDP from the peer
sudo ufw allow proto udp from <PEER_IP>
```

#### 3. Common configuration mistakes

| Issue | Symptom | Fix |
|-------|---------|-----|
| Domain ID mismatch | No topics discovered | Ensure `Domain Id` in XML matches `ROS_DOMAIN_ID` env var on both sides |
| Wrong NetworkInterface | CycloneDDS binds to wrong interface | Set `address=` to an IP that exists on this machine (`ip a` to check) |
| Missing peer | No discovery | Each side's `<Peer>` must point to the **other** machine's IP |
| MaxMessageSize mismatch | Image corruption | Use `65500B` on both sides |
| `RMW_IMPLEMENTATION` not set | Uses default FastDDS | Must be `rmw_cyclonedds_cpp` on both sides |

#### 4. VPN-specific issues

When connecting through a VPN (e.g., Cisco AnyConnect):

- Use the **VPN tunnel IP** (from `cscotun0`) as the NetworkInterface address, not the WiFi IP
- The VPN IP is **dynamic** — update `cyclonedds.xml` on both sides after each VPN reconnect

#### 5. Quick end-to-end ROS 2 test

```bash
# Server:
ros2 run demo_nodes_cpp talker

# Client:
ros2 run demo_nodes_cpp listener
# Should print: "I heard: [Hello World: X]"
```

---

## 📚 Additional Resources

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Isaac Sim ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Isaac Sim ROS 2 Tutorials](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html)
- [TidyBot++ Project](https://tidybot2.github.io/)
