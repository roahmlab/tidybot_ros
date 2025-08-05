# tidybot_utils
## Overview
This package defines the tidybot custom message and service types

## WSMsg
This message type contains the command received from the webapp

- uint64 timestamp
- string state_update
- string device_id
- string teleop_mode
- float64 pos_x
- float64 pos_y
- float64 pos_z
- float64 or_x
- float64 or_y
- float64 or_z
- float64 or_w
- float64 gripper_delta

## ResetEnv
This service type defines the reset environment service
- Request: bool reset
- Response: bool success