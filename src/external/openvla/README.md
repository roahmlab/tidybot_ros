# tidybot_openvla
## 📖 Overview
This package wraps the core OpenVLA source code, allowing deployment of fine-tuned models on hardware. It enables the receival of input images and the publishing of inferred actions as end effector deltas over ROS topics defined in tidybot_driver.

## 🎯 Key Features
TODO

## 📁 Package Structure

```
tidybot_openvla/
├── openvla/                            # Original openVLA source code
│   ├── ...
│   ├── tidybot_openvla.py              # Launches a node to listen to published images, run inference on a fine-tuned model, and publish actions back to hardware
```

## 🚀 Communication Setup
To enable communication between the Tidybot's on-board computer and a remote compute server, ensure the following on both machines:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=$(realpath cyclonedds.xml)
```
Note that FastDDS is an alternative option, but published images are fragmented before sending, leading to the received images being corrupted. CycloneDDS led to the most reliable results during our testing, and only requires setting up the following configurations on the robot and server side. 

Robot Side:
```xml
<?xml version="1.0" encoding="utf-8"?>
<CycloneDDS
	  xmlns="https://cdds.io/config"
	  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
	  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd"
	>
  <Domain Id="1">
    <General>
      <Interfaces>
	      <NetworkInterface address="<!-- Robot IP -->"/> 
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <Peers>
        <Peer address="<!-- Server IP -->"/> 
      </Peers>
    </Discovery>
    <Tracing>
      <Verbosity>config</Verbosity>
      <OutputFile>${HOME}/dds/log/cdds.log.${CYCLONEDDS_PID}</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>

```

Server Side:
```xml
<?xml version="1.0" encoding="utf-8"?>
<CycloneDDS
	  xmlns="https://cdds.io/config"
	  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
	  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd"
	>
  <Domain Id="1">
    <General>
      <Interfaces>
      <NetworkInterface autodetermine="true" priority="default" multicast="false"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
	<Peers>
      	</Peers>
    </Discovery>
    <Tracing>
      <Verbosity>config</Verbosity>
      <OutputFile>${HOME}/dds/log/cdds.log.${CYCLONEDDS_PID}</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
```

## 🎛️ Nodes

### **OpenVLA Inference (`tidybot_openvla_node`)**
Loads checkpoints for the original OpenVLA model, plus fine-tuned parameters using peft and unnormalization statistics based on the finetuning dataset. Then, listens to incoming images from `tidybot/camera_ext/color/compressed` or `tidybot/camera_wrist/color/compressed` on its own thread, which continuously updates the current image. On a separate thread, performs inference on the most recent image and publishes the end effector action continuously to `tidybot/arm/delta_commands` at 15Hz. Every 5Hz, inference is performed again on the most recent image, which updates the published action.

```bash
ros2 run tidybot_openvla openvla_node
```

**Subscribed Topics:**
- `/tidybot/hardware/arm/camera_ext` (sensor_msgs/CompressedImage): Observed 224x224 resolution compressed jpeg

**Published Topics:**
- `/tidybot/arm/delta_commands` (std_msgs/Float64MultiArray): Inferred end effector action in the form of delta position, delta rotation and gripper state

**Features:**
- **Action Decoding to Continuous Trajectory**: By publishing the end effector delta continuously, which updates an accumulated target position in the `moveit_ee_pose_ik` node, this allows the robot to execute actions as continuous trajectories with each inference.

## 🐛 Troubleshooting

### **Common Issues**

TODO

## 🔗 Dependencies

TODO

## 📚 Additional Resources

TODO
