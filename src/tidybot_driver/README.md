# tidybot_driver
## Overview
This package contains the interface for the tidybot hardware: the base, arm cameras.

## driver.launch.py
This file launches the nodes required for controlling and interfacing the tidybot hardware and expose topics to receive the control command.

Parameter:
- mode: control mode of the robot, control the whole robot if mode:=full, only the base if mode:=base_only, only the arm if mode:=arm_only