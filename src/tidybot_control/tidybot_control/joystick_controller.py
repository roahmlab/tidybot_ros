import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
# ServoCommandType import removed - no longer needed with C++ API
from sensor_msgs.msg import JointState, Joy
from scipy.spatial.transform import Rotation as R
from tf_transformations import quaternion_multiply, quaternion_inverse
import tf2_ros
from tf2_ros import TransformBroadcaster
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.time import Time

import numpy as np
import math
import gc
import threading
from queue import Queue
import threading
from enum import Enum

"""Joystick Message mapping:
- Axes:
  - 0: Left stick horizontal (base x | arm linear y)
  - 1: Left stick vertical (base y | arm linear z)
  - 2: Left trigger (arm roll positive)
  - 3: Right stick horizontal (base theta | arm angular yaw)
  - 4: Right stick vertical (base y | arm angular pitch)
  - 5: Right trigger (arm roll negative)
  - 6: D-pad horizontal (gripper open/close)
  - 7: D-pad vertical (arm linear x)
- Buttons:
  - 0: A button (arm mode)
  - 1: B button (base mode)
  - 2: X button (arm base frame)
  - 3: Y button (arm end-effector frame)
  - 4: Left bumper (enable)
  - 5: Right bumper (enable)
  - 6: Overview button
  - 7: Menu button
  - 8: Manufacturer button (next state)
- Additional buttons can be mapped as needed.
"""

GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
RED = "\x1b[31m"
RESET = "\x1b[0m"
BOLD = "\x1b[1m"

class ControlMode(Enum):
    BASE = 0
    ARM = 1

class ArmControlFrame(Enum):
    BASE = 0
    EE = 1

class JoystickController(Node):
    def __init__(self):
        super().__init__("joystick_controller")
        self.declare_parameter("use_sim", True)
        self.use_sim = self.get_parameter("use_sim").get_parameter_value().bool_value
        self.declare_parameter("control_frequency", 20.0)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )
        
        if self.use_sim:
            base_topic = "/tidybot_base_vel_controller/commands"
        else:
            base_topic = "/tidybot/hardware/base/target_vel"
        self.base_pub = self.create_publisher(Float64MultiArray, base_topic, 10)

        self.control_enabled = False
        self.control_mode = None
        self.arm_control_frame = ArmControlFrame.EE

        self.gripper_state = 0.0  # initial gripper state (opened)
        self.last_control_enabled = False
        
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        
        self.arm_pub = self.create_publisher(
            TwistStamped, "/tidybot/arm/twist", 10
        )
        if self.use_sim:
            self.gripper_pub = self.create_publisher(
                Float64MultiArray, "/robotiq_2f_85_controller/commands", 10
            )
        else:
            self.gripper_pub = self.create_publisher(
                Float64, "/tidybot/hardware/gripper/commands", 10
            )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        threshold = JumpThreshold(
            min_forward=None, min_backward=Duration(seconds=-1), on_clock_change=True
        )
        self.jump_handler = self.get_clock().create_jump_callback(
            threshold, post_callback=self.time_jump_callback
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_callback
        )

        self.cmd_queue = Queue(maxsize=1)
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def joy_callback(self, msg: Joy):
        if self.cmd_queue.full():
            self.cmd_queue.get()
        hardware_enabled = msg.buttons[4] != 0 or msg.buttons[5] != 0
        if hardware_enabled and not self.control_enabled:
            self.control_enabled = True
            self.get_logger().info(f"{GREEN}Joystick control enabled{RESET}")
        elif not hardware_enabled and self.control_enabled:
            self.control_enabled = False
            self.get_logger().info(f"{RED}Joystick control disabled{RESET}")
        if msg.buttons[0] == 1 and msg.buttons[1] == 0 and self.control_mode != ControlMode.ARM:
            self.control_mode = ControlMode.ARM
            self.get_logger().info(f"{GREEN}Switch to ARM mode{RESET}")
        elif msg.buttons[1] == 1 and msg.buttons[0] == 0 and self.control_mode != ControlMode.BASE:
            self.control_mode = ControlMode.BASE
            self.get_logger().info(f"{GREEN}Switch to BASE mode{RESET}")
        if msg.buttons[2] == 1 and msg.buttons[3] == 0 and self.arm_control_frame != ArmControlFrame.BASE:
            self.arm_control_frame = ArmControlFrame.BASE
            self.get_logger().info(f"{GREEN}Switch to ARM BASE frame{RESET}")
        elif msg.buttons[3] == 1 and msg.buttons[2] == 0 and self.arm_control_frame != ArmControlFrame.EE:
            self.arm_control_frame = ArmControlFrame.EE
            self.get_logger().info(f"{GREEN}Switch to ARM END-EFFECTOR frame{RESET}")
        self.cmd_queue.put(msg)

    def time_jump_callback(self, time: Time):
        # re-instantiate the TF listener and buffer to avoid TF_OLD_DATA warning
        self.destroy_subscription(self.tf_listener.tf_sub)
        self.destroy_subscription(self.tf_listener.tf_static_sub)
        del self.tf_listener
        del self.tf_buffer
        gc.collect()
        # Clear both dynamic and static entries
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF buffer reset")

    def control_callback(self):
        if self.control_thread.is_alive():
            self.get_logger().warning("Control thread is running")
        else:
            self.control_thread = threading.Thread(target=self.control_loop)
            self.control_thread.start()

    def control_loop(self):
        base_cmd = Float64MultiArray()
        arm_cmd = TwistStamped()
        base_cmd.data = [0.0, 0.0, 0.0]
        arm_cmd.header.stamp = self.get_clock().now().to_msg()
        if self.arm_control_frame == ArmControlFrame.BASE:
            arm_cmd.header.frame_id = "arm_base_link"  # relative to the base frame
        else:
            arm_cmd.header.frame_id = "bracelet_link"  # relative to the end-effector frame     
        arm_cmd.twist.linear.x = 0.0
        arm_cmd.twist.linear.y = 0.0
        arm_cmd.twist.linear.z = 0.0
        arm_cmd.twist.angular.x = 0.0
        arm_cmd.twist.angular.y = 0.0
        arm_cmd.twist.angular.z = 0.0
        if not self.control_enabled:
            if self.last_control_enabled:
                # Send a single zero command to stop hardware safely
                self.base_pub.publish(base_cmd)
                self.arm_pub.publish(arm_cmd)
                self.publish_gripper()
            self.last_control_enabled = False
            return

        self.last_control_enabled = True
        cmd = self.cmd_queue.get() if not self.cmd_queue.empty() else None

        if self.control_mode == ControlMode.BASE:
            base_cmd_raw = np.array([
                cmd.axes[1] * 0.2 if cmd else 0.0,
                cmd.axes[0] * 0.2 if cmd else 0.0,
                cmd.axes[3] * 0.2 if cmd else 0.0,
            ], dtype=np.float64)
            base_cmd.data = base_cmd_raw.tolist()
            self.base_pub.publish(base_cmd)

        elif self.control_mode == ControlMode.ARM:
            linear = np.array([
                cmd.axes[0] if cmd else 0.0,
                -(cmd.axes[1]) if cmd else 0.0,
                -(cmd.axes[7]) if cmd else 0.0,
            ], dtype=np.float64)
            angular = np.array([
                -(cmd.axes[4]) if cmd else 0.0,
                -(cmd.axes[3]) if cmd else 0.0,
                (-(cmd.axes[2] - 1) + (cmd.axes[5] - 1)) / 2 if cmd else 0.0,
            ], dtype=np.float64)

            arm_cmd.twist.linear.x = linear[0]
            arm_cmd.twist.linear.y = linear[1]
            arm_cmd.twist.linear.z = linear[2]
            arm_cmd.twist.angular.x = angular[0]
            arm_cmd.twist.angular.y = angular[1]
            arm_cmd.twist.angular.z = angular[2]

            # Gripper control
            if cmd:
                if cmd.axes[6] > 0.1:  # D-pad right
                    self.gripper_state = min(self.gripper_state + 0.02, 0.81)  # open
                elif cmd.axes[6] < -0.1:  # D-pad left
                    self.gripper_state = max(self.gripper_state - 0.02, 0.0)  # close
            self.arm_pub.publish(arm_cmd)
            self.publish_gripper()

    def publish_gripper(self):
        if self.use_sim:
            msg = Float64MultiArray()
            msg.data = [float(self.gripper_state)]
        else:
            msg = Float64()
            msg.data = float(self.gripper_state)
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
