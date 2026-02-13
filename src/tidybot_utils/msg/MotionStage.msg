# Stage type
uint8 STAGE_PTP = 0      # Point-to-point motion (free-space)
uint8 STAGE_LIN = 1      # Linear constraint motion (straight line)
uint8 STAGE_CIRC = 2     # Circular constraint motion (arc)
uint8 STAGE_GRIPPER = 3  # Gripper action
uint8 STAGE_WAIT = 4     # Wait/hold position for duration
uint8 stage_type

# Target pose (for PTP, LIN, CIRC endpoint)
geometry_msgs/Pose target_pose

# For CIRC: arc center point and axis
geometry_msgs/Point arc_center
geometry_msgs/Vector3 arc_axis
float64 arc_angle                     # radians, positive = CCW around axis

# For GRIPPER action
float64 gripper_position              # 0.0 = open, 1.0 = closed

# Motion parameters
float64 velocity_scaling              # 0.0 to 1.0
float64 duration                      # estimated duration in seconds
string description                    # for feedback/logging
