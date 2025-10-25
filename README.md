# TidyBot++ ROS2 Platform

A comprehensive ROS 2 platform for controlling the [TidyBot++](https://tidybot2.github.io/) mobile manipulator robot. This platform provides simulation, hardware control, teleoperation, and data collection capabilities for a mobile robot equipped with a Kinova Gen3 7-DOF arm and Robotiq 2F-85 gripper.

## 🤖 Robot Overview

The TidyBot++ is a mobile manipulator consisting of:
- **Mobile Base**: Omnidirectional platform with holonomic drive
- **Manipulator**: Kinova Gen3 7-DOF robotic arm
- **End Effector**: Robotiq 2F-85 parallel gripper
- **Sensors**: Integrated camera system for vision-based tasks

## 🏗️ System Architecture
TODO
## 📦 Package Descriptions

### Core Packages

#### `tidybot_description`
Robot model definition and simulation setup.
- URDF/XACRO files for complete robot description
- Gazebo simulation configuration
- RViz visualization setup
- Controller configuration files

#### `tidybot_driver` 
Hardware interface for real robot control.
- Kinova arm driver integration
- Mobile base control interface
- Camera streaming capabilities
- Joint state publishing and command handling

#### `tidybot_control`
Teleoperation and remote control systems.
- WebXR-based smartphone teleoperation
- Remote policy server integration
- Real-time control command processing
- Episode recording integration

#### `tidybot_solver`
Motion planning and control integration.
- MoveIt2 integration for arm planning
- Real-time servo control capabilities
- Trajectory execution and monitoring

#### `tidybot_moveit_config`
MoveIt2 configuration package (auto-generated).
- Motion planning configuration
- Kinematics solver setup
- Collision detection parameters
- Planning scene configuration

#### `tidybot_episode`
Data recording and replay system.
- ROS bag recording of control episodes
- Data conversion to HDF5 format
- Dataset generation for machine learning

#### `tidybot_utils`
Shared utilities and message definitions.
- Custom message types for teleoperation
- Service definitions for environment reset
- Utility functions and constants

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 24.04 LTS
- **ROS**: ROS 2 Jazzy
- **Python**: 3.10+
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

#### Option 1: Docker (Recommended)

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
Then in another **host** terminal:
```bash
# Setup the CAN connection
sudo bash ./scripts/setup_docker_can.sh
```

#### Option 2: Native Installation
TODO
## 🎮 Example usage

### Choose a robot to play with

### 1. Simulated robot
Perfect for development and testing without hardware.
```bash
ros2 launch tidybot_description launch_sim_robot.launch.py
```
- Launch the Gazebo simulation environment and publish corresponding topics for robot control and monitoring
- Launch RViz2 for robot visualization and camera view

### 2. Physical robot
Direct control of the physical robot.
```bash
# Start hardware drivers
ros2 launch tidybot_driver launch_hardware_robot.launch.py mode:=full
```

### Choose a control mode

### 1. Phone Teleop mode
Publish the WebXR app for phone and tablet to connect and relay the web messages to the robot
```bash
ros2 launch tidybot_control teleop.launch.py use_sim:=true
```

### 2. Joystick mode
Connect to the gamepad and relay joystick messages to the robot
```bash
ros2 launch tidybot_control joystick.launch.py use_sim:=true
```

### 3. Remote mode
TODO

## 📊 Data Collection

The platform supports comprehensive data collection for machine learning:

- **Observations**: Camera feeds, joint states, end-effector poses
- **Actions**: Control commands, trajectory waypoints
- **Episodes**: Complete task executions with start/end markers
- **Export Formats**: ROS bags, HDF5, custom formats

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

## 📄 License

TODO

## 📞 Support

- **Maintainers**: 
  - janchen@umich.edu
  - yuandi@umich.edu
- **Issues**: Create GitHub issues for bug reports and feature requests
- **Documentation**: See individual package READMEs for detailed information

## 🔗 Related Projects

- [The original Tidybot++ project](https://tidybot2.github.io/)
- [MoveIt2](https://moveit.ros.org/)
- [Kinova Gen3 ROS2](https://github.com/Kinovarobotics/ros2_kortex)

---

*TidyBot++ Platform - Enabling advanced mobile manipulation research and applications*