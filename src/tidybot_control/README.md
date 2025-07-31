# tidybot_control
This package handles the remote control for the robot and is expected to run alongside with the simulation package or hardware driver package.

## teleop.launch.py
This launch file launches the nodes required by phone teleoperation. When the nodes in this launch file are running, a WebXR webapp will be published at the host's ip address on port 5000. Press the 'Start episode' button on the website to start an episode. After an episode is started, press the left edge of the screen to initiate base control; press the middle of the screen to initiate arm control; press the middle of the phone and slide upward to close the gripper and slide downward to open the gripper. When teleoperating the robot, the phone's pose will be transformed to robot's frame and the end effector / the base will follow the motion of the phone. Press 'End episode' to stop recording the episode. Press 'Reset env' to reset the environment.

Parameter: use_sim - specify the robot to control, when true the teleop commands will be sent to the simulated robot.

## remote.launch.py
This launch file launches the nodes required by remote policy server control. When the nodes are running, a WebXR webapp will be published as teleop.launch.py to serve as the switch for the remote control. Press the 'Start episode' and press on the screen to enable remote control and start receiving command from the policy server. Then press 'End episode' to stop remote control and switch to teleop control, the user can then control the robot as teleop.launch.py. Press 'Reset env' to reset the environment and also the remote server.

Parameter: use_sim - specify the robot to control, when true the teleop commands will be sent to the simulated robot.