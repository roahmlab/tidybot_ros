# TidyBot++ ROS2 Platform

A comprehensive ROS 2 platform for controlling the [TidyBot++](https://tidybot2.github.io/) mobile manipulator. This codebase integrates a policy learning pipeline into both simulated and real environments, and encapsulates all communication using ROS 2 nodes. Developed at the [ROAHM Lab](https://www.roahmlab.com/) at the University of Michigan, Ann Arbor.

This project is an adaptation of the original TidyBot++ project (Wu et. al., full citation below) and other open-source projects. 

## 📞 **Maintainers**:
- Zhuoyang Chen (janchen@umich.edu)
- Yuandi Huang (yuandi@umich.edu)

## 🤖 Robot Overview

The TidyBot++ is a mobile manipulator consisting of:
- **Mobile Base**: Omnidirectional platform with holonomic drive
- **Manipulator**: Kinova Gen3 7-DOF robotic arm
- **End Effector**: Robotiq 2F-85 parallel gripper
- **Sensors**: RGBD wrist/base-mounted cameras, optional external cameras

A complete bill of materials and hardware setup guide is available in the [original project documentation](https://tidybot2.github.io/docs/).

## 🔢 Policy Deployment Demos
Using this codebase, we have successfully trained diffusion policies and finetuned vision-language-action models. Below are demonstrations of policies deployed in our simulated environment and on hardware:

| <img src="https://github.com/user-attachments/assets/d778a6cd-1bce-4195-b2b0-d4013957151f" width="325"/> | <img src="https://github.com/user-attachments/assets/8d7b99dc-de36-460f-b997-3077c3b44844" width="530"/> | <img src="https://github.com/user-attachments/assets/abe826a9-dc3a-4d32-8665-148473e8a45c" width="530"/> | <img src="https://github.com/user-attachments/assets/3e32cacb-5390-4ee4-98e4-95cc699a7657" width="530"/> | 
|:--:|:--:|:--:|:--:|
| Diffusion policy in Gazebo Sim | Diffusion policy in Isaac Sim | Diffusion policy on hardware | VLA on hardware |


## 🏗️ System Architecture
![Image](https://github.com/user-attachments/assets/097e7578-3f0e-4434-8396-53b5bb39c490)

## 📦 Core Packages

#### [`tidybot_description`](./src/tidybot_description/README.md)
Robot model definition and simulation setup.
- URDF/XACRO files for complete robot description
- Configurations for Gazebo simulation
- Setup for RViz visualization 

## 🖥️ Isaac Sim Integration

This workspace supports high-fidelity simulation using NVIDIA Isaac Sim. The integration uses a dual-container architecture:
1.  **TidyBot Container**: Runs the ROS 2 control stack.
2.  **Isaac Sim Container**: Runs the simulator with the ROS 2 Bridge extension.

The two containers communicate over the host network using FastDDS.

For detailed setup and usage instructions, please refer to:
*   **[Docker Setup](./docker/README.md)**: Instructions for building and running the Isaac Sim container.
*   **[Simulation Setup](./src/tidybot_description/README.md#%F0%9F%8E%AE-isaac-sim-integration)**: Detailed steps for importing the robot, configuring the ActionGraph, and running the setup
script in Isaac Sim.
*   **[Policy Deployment](./src/tidybot_policy/README.md#%F0%9F%8E%AE-policy-deployment)**: Detailed steps  for teleoperating the robot and deploying policies in Isaac Sim.


#### [`tidybot_driver`](./src/tidybot_driver/README.md)
Low-level controllers for hardware.
- Kinova Kortex API integration
- Mobile base control interface
- Publishers for camera feeds
- Joint state publisher

#### [`tidybot_policy`](./src/tidybot_policy/README.md)
Teleoperation and policy deployment.
- WebXR-based smartphone teleoperation
- Gamepad-based teleoperation
- Remote inference integration
- Real-time control command processing
- Episode recording integration

#### [`tidybot_solver`](./src/tidybot_solver/README.md)
Motion planning and inverse kinematics. We integrate the original TidyBot++ WebXR teleoperation interface with MoveIt2.
- MoveIt2 integration for arm planning
- Real-time servo control capabilities
- Execution of trajectories on low-level controllers

#### [`tidybot_moveit_config`](./src/tidybot_moveit_config/README.md)
MoveIt2 configuration package (auto-generated from robot description).
- Motion planning configuration
- Kinematics solver setup
- Collision detection parameters
- Planning scene configuration

#### [`tidybot_episode`](./src/tidybot_episode/README.md)
Data recording and replay system.
- ROS bag recording of teleoperation episodes
- Data conversion to HDF5 format or .parquet
- Dataset generation for policy training

#### [`tidybot_utils`](./src/tidybot_utils/README.md)
Custom messages and services for teleoperation

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 24.04 LTS
- **Docker**: Latest version (for containerized deployment)

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/roahmlab/tidybot_platform.git
   cd tidybot_platform
   ```

2. **Build the workspace:**
   ```bash
   ./docker/build.sh
   ```

### Running the System

Connect the Canivore-usb module, the Kinova Gen3 arm and Orbbec camera to the dev machine.

Before launching the docker container:
```bash
# Setup the base camera access
sudo bash ./scripts/install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Then run the container and setup the environment inside the container:
```bash
# Run the container
./docker/run.sh restart

# --------------------------Inside the container-------------------------- #

# Install the canivore-usb inside the container
sudo apt update && sudo apt install canivore-usb -y

# Create a python virtual environment
python3 -m venv --system-site-packages venv && source venv/bin/activate

# Install all required python packages
pip install -r requirements.txt

# Install pre-built OpenCV wheel
wget -O opencv_python-4.9.0.80-cp312-cp312-linux_x86_64.whl "https://www.dropbox.com/scl/fi/mzfz0i7qljmwzm5bm22td/opencv_python-4.9.0.80-cp312-cp312-linux_x86_64.whl?rlkey=rbero1ycahdk1d6jeixiu5p5l&st=k5dzfv6a&dl=0" && pip install --force-reinstall --no-deps opencv_python-4.9.0.80-cp312-cp312-linux_x86_64.whl && rm ./opencv_python-4.9.0.80-cp312-cp312-linux_x86_64.whl

# Install Kortex-API
wget https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl && pip install ./kortex_api-2.6.0.post3-py3-none-any.whl && pip install protobuf==3.20.0 && rm ./kortex_api-2.6.0.post3-py3-none-any.whl

# Ignore venv when colcon build
touch venv/COLCON_IGNORE

# Source the environment
colcon build && source install/setup.bash && export PYTHONPATH=$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH
```

To add venv path to your PYTHONPATH automatically every time you start a new session, use

```bash
echo 'export PYTHONPATH=$HOME/tidybot_platform/venv/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc
```

Then in another **host** terminal:
```bash
# Setup the CAN connection
sudo bash ./scripts/setup_docker_can.sh
```

## 🎮 Example usage

### Choose a robot environment

#### 1. Simulated robot (Gazebo)
```bash
ros2 launch tidybot_description launch_sim_robot.launch.py base_mode:=velocity
```

#### 2. Simulated robot (Isaac Sim)
Ensure Isaac Sim is running with the robot loaded (see [Simulation Setup](#-isaac-sim-integration)).
```bash
ros2 launch tidybot_description launch_isaac_sim.launch.py use_velocity_control:=true
```

#### 3. Physical robot
```bash
# Start hardware drivers
ros2 launch tidybot_driver launch_hardware_robot.launch.py mode:=full
```

### Choose a control mode

#### 1. Phone Teleop mode
Publish the WebXR app for phone and tablet to connect and relay the web messages to the robot.
```bash
# For real robot
ros2 launch tidybot_policy launch_phone_policy.launch.py sim_mode:=hardware

# For Gazebo simulation
ros2 launch tidybot_policy launch_phone_policy.launch.py sim_mode:=gazebo

# For Isaac Sim
ros2 launch tidybot_policy launch_phone_policy.launch.py sim_mode:=isaac
```

#### 2. Gamepad mode
Connect to an Xbox Series X gamepad and relay joystick messages to the robot.
```bash
ros2 launch tidybot_policy launch_gamepad_policy.launch.py sim_mode:=<hardware|gazebo|isaac>
```

### Collect demonstrations for Diffusion Policy training

#### 1. Setup the data collection pipeline
Follow the previous section on how to launch a robot. Then launch the teleop with recording enabled.

```bash
ros2 launch tidybot_policy launch_phone_policy.launch.py sim_mode:=<hardware|gazebo|isaac> record:=true
```
This launch file should publish a webserver that will prompt the user to save/discard the recorded episode after the episode ends. If such a window does not pop up, try to refresh the webpage.

The recorded episodes will be stored in `episode_bag` folder in the current directory as ros bags.

#### 2. Convert the ros bags to compatible dataset
After the data collection is done. Run the converted node to convert all the rosbags into a .hdf5 tarball.

```bash
ros2 run tidybot_episode rosbag_to_hdf5
```
The converted dataset will be saved as `data.hdf5` under the current directory

### Policy Training
We recommend going to the original [Tidybot++](https://github.com/jimmyyhwu/tidybot2?tab=readme-ov-file#policy-training) codebase to train a diffusion policy for the tidybot platform. Our platform generates datasets with an identical structure to the original project.

### Policy Inference
The policy server in this part is adapted from the original Tidybot++ project. You can follow the same instructions from the original [documentation](https://github.com/jimmyyhwu/tidybot2?tab=readme-ov-file#policy-inference) to setup the policy server on a GPU machine. Once the GPU server is running, setup a SSH tunnel from the dev machine to GPU server by
```bash
ssh -L 5555:localhost:5555 <gpu-server-hostname>
```
Then launch the remote policy on the dev machine by
```bash
ros2 launch tidybot_policy launch_remote_policy_diffusion.launch.py sim_mode:=<hardware|gazebo|isaac>
```
This launch file will also launch a webserver that can be used to control the policy inference. By pressing the middle of the screen, the inference will be enabled. If there's no user interaction detected on the webserver, the inference will be paused.

## 🔧 Configuration

### Robot Configuration
- Modify `src/tidybot_description/config/tidybot_controllers.yaml`
- Adjust joint limits in `src/tidybot_moveit_config/config/joint_limits.yaml`

### Network Configuration
- Default WebXR port: 5000
- ROS domain ID can be set via `ROS_DOMAIN_ID` environment variable

### Hardware Configuration
- CAN bus setup: `./setup_can.sh` if running inside of a docker container
- Camera configuration in driver package

## 🐛 Troubleshooting

### Common Issues

1. **Build Failures**
   ```bash
   # Clean and rebuild
   rm -rf build/ install/ log/
   ./build.sh
   ```

2. **Network Issues**
   - Ensure firewall allows port 5000
   - Check ROS_DOMAIN_ID consistency
   - Verify network connectivity between devices

3. **Hardware Connection Issues**
   - Check CAN bus connection: `candump can0`

4. **Docker Container Issues**
   - If the Rviz / Gazebo simulation viewer does not launch, try to establish X11 permissions/Xauthority on the host before starting the container:
      ```bash
      xhost +SI:localuser:$(whoami)
      xhost +SI:localuser:root
      ```
      If you are on Wayland, export an X display first:
      ```bash
      export DISPLAY=${DISPLAY:-:0}
      ```
      Then restart the container.
   

### Logs and Debugging
- ROS logs: `~/.ros/log/`
- Container logs: `docker logs tidybot_platform`
- Enable debug mode: Set `ROS_LOG_LEVEL=DEBUG`

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request



## 🔗 Related Projects

- [The original Tidybot++ project](https://tidybot2.github.io/)
   ```bibtex
   @inproceedings{wu2024tidybot,
    title = {TidyBot++: An Open-Source Holonomic Mobile Manipulator for Robot Learning},
    author = {Wu, Jimmy and Chong, William and Holmberg, Robert and Prasad, Aaditya and Gao, Yihuai and Khatib, Oussama and Song, Shuran and Rusinkiewicz, Szymon and Bohg, Jeannette},
    booktitle = {Conference on Robot Learning},
    year = {2024}
  }
- [MoveIt2](https://moveit.ros.org/)
- [Kinova Gen3 ROS2](https://github.com/Kinovarobotics/ros2_kortex)
- [OrbbecSDK ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)
- [Diffusion Policy](https://github.com/real-stanford/diffusion_policy)
---
