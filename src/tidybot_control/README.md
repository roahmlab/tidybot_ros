# tidybot_control

## 📖 Overview

This package provides advanced teleoperation and remote control capabilities for the TidyBot++ mobile manipulator. It features a WebXR-based interface that enables intuitive smartphone control, remote policy server integration for AI-driven control and joystick control interface for test and debugging

## 🎮 Control Modes

### **WebXR Teleoperation**
Intuitive smartphone-based control using device orientation and touch input.
- **Immersive Control**: Uses phone's IMU for natural robot control
- **Multi-modal Interface**: Separate controls for base, arm, and gripper
- **Real-time Feedback**: Live camera feeds and robot state visualization
- **Episode Management**: Integrated recording and replay capabilities

### **Remote Policy Control**
AI/ML policy server integration with human oversight capabilities.
- **Policy Server Interface**: Connects to external AI control systems
- **Human Override**: Seamless switching between AI and manual control
- **Safety Monitoring**: Real-time safety checks and emergency stops
- **Data Collection**: Automatic logging of policy decisions and outcomes

### **Joystick Control**
Traditional gamepad-based control for development and testing.
- **Standard Gamepad Support**: Xbox Series X Controller
- **Configurable Mapping**: Customizable button and axis assignments
- **Precision Control**: Fine-grained control over robot movements
- **Emergency Controls**: Dedicated safety buttons and stops

## 🚀 Launch Files

### `teleop.launch.py`
Launch WebXR-based smartphone teleoperation system.

```bash
# Launch with simulation
ros2 launch tidybot_control teleop.launch.py use_sim:=true

# Launch with real hardware
ros2 launch tidybot_control teleop.launch.py use_sim:=false
```

**Parameters:**
- `use_sim` (bool): Target simulation (true) or real hardware (false)

**Access WebXR Interface:**
- URL: `http://(master machine ip address):5000`
- Compatible with smartphones, tablets, and VR headsets
- Requires HTTPS for full WebXR features

**Control Interface:**
- **Start Episode**: Begin recording and control session
- **Right Screen Edge**: Activate base control mode
- **Center Screen**: Activate arm control mode
- **Vertical Swipe**: Control gripper (up=close, down=open)
- **End Episode**: Stop recording and save data
- **Reset Environment**: Reset robot to initial state

### `remote.launch.py`
Launch remote policy server control with human oversight.

```bash
# Launch remote policy mode
ros2 launch tidybot_control remote.launch.py use_sim:=true
```

**Parameters:**
- `use_sim` (bool): Target simulation (true) or real hardware (false)

**Features:**
- **Policy Server Integration**: Connects to external AI control systems
- **Human Override**: WebXR interface remains available for manual intervention
- **Safety Monitoring**: Continuous safety checks and automatic stops
- **Data Logging**: Records policy decisions and human interventions

### `joystick.launch.py`
Launch gamepad-based control system.

```bash
# Launch joystick control
ros2 launch tidybot_control joystick.launch.py
```

**Supported Controllers:**
- Xbox Series X controller