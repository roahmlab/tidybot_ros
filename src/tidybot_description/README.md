# tidybot_description

## Overview
This package contains resources for the tidybot++ model and defines how to spawn the tidybot++ in Gazebo, which world the robot is spawned to.

## description.launch.py
This is the minimum launch file to launch the robot_state_publisher for the urdf model

Parameters: 
- robot_description: the path to the urdf model to be spawned, the default is the tidybot's urdf
- use_sim_time: use sim time if true
- ignore_timestamp: ignore the time stamp in the robot_state_publisher if true

## display.launch.py
This file launched the rviz visualization for the urdf model and (optional) a joint_state_publisher gui to control the joints in the model

Parameters:
- jsp_gui: use joint_state_publisher gui if true
- rviz_config: the path to the rviz configuration file, default is the tidybot.rviz
- robot_description_content: the path to the urdf model, defaul is the tidybot model
- use_sim_time: use simulation time if true

## sim.launch.py
This file launches the gazebo simulation and spawns the robot into gazebo. A rviz window (optional) for the robot can also be launched alongside with the simulation to visualize the tf of the robot and the camera views 

Parameters:
- use_rviz: launch rviz if true, default=true
- rviz_config: path to the rviz configuration file, default is tidybot.rviz
- base_mode: control mode for the base, use position control (used by teleop control) if base_mode:=position, use velocity control (used by joystick control) if base_mode:=velocity
- world: the world in which the robot will be spawned, default is empty 

