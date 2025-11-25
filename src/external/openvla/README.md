## Finetuning Infrastructure
To register the RLDS dataset for training on, we provide an example `configs.py` and `transforms.py` to integrate with the openvla source code. These scripts should be placed [here](https://github.com/openvla/openvla/blob/main/prismatic/vla/datasets/rlds/oxe/configs.py#L54) and [here](https://github.com/openvla/openvla/blob/main/prismatic/vla/datasets/rlds/oxe/transforms.py#L828), respectively. 

## Communication Setup
To enable communication between the Tidybot's on-board computer and a remote compute server, ensure both machines have the same ROS_DOMAIN_ID and have cyclonedds:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=$(realpath cyclonedds.xml)
```
FastDDS is an alternative option, but the published images are fragmented before sending, leading to the received images being corrupted. We use CycloneDDS which requires setting up the following configurations on the robot and server side. 

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

### **OpenVLA Inference (`tidybot_openvla_node`)**
Loads checkpoints for the original OpenVLA model, plus fine-tuned parameters using peft and unnormalization statistics based on the finetuning dataset. Then, listens to incoming images from `tidybot/camera_ext/color/compressed` or `tidybot/camera_wrist/color/compressed` on its own thread, which continuously updates the current image. On a separate thread, performs inference on the most recent image and publishes the end effector action continuously to `tidybot/arm/delta_commands` at 15Hz. Every 5Hz, inference is performed again on the most recent image, which updates the published action.

```bash
ros2 run tidybot_policy openvla_node
```

**Subscribed Topics:**
- `/tidybot/hardware/arm/camera_ext` (sensor_msgs/CompressedImage): Observed 224x224 resolution compressed jpeg

**Published Topics:**
- `/tidybot/arm/delta_commands` (std_msgs/Float64MultiArray): Inferred end effector action in the form of delta position, delta rotation and gripper state