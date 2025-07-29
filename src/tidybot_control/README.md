# tidybot_control
This package handles the remote control for the robot and is expected to run alongside with the simulation package or hardware driver package.

## teleop.launch.py
This launch file launches the nodes required by phone teleoperation. When the nodes in this launch file are running, a WebXR webapp will be published at the host's ip address on port 5000. Press the 'start_episode' button on the website to start an episode. After an episode is started, press the left edge of the screen to initiate base control; press the middle of the screen to initiate arm control; press the middle of the phone and slide upward to close the gripper and slide downward to open the gripper. When teleoperating the robot, the phone's pose will be transformed to robot's frame and the end effector will mimic the phone's pose.

Parameter: use_sim - specify the robot to control, when true the teleop commands will be sent to the simulated robot.