import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
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
  - 0: Left stick horizontal (base x | arm linear x)
  - 1: Left stick vertical (base y | arm linear y)
  - 2: Left trigger 
  - 3: Right stick horizontal (base theta | arm angular x)
  - 4: Right stick vertical (base y | arm angular y)
  - 5: Right trigger 
  - 6: D-pad horizontal (arm linear z)
  - 7: D-pad vertical (arm angular z)
- Buttons:
  - 0: A button (arm mode)
  - 1: B button (base mode)
  - 2: X button 
  - 3: Y button 
  - 4: Left bumper (enable)
  - 5: Right bumper (enable)
  - 6: Overview button (reset arm)
  - 7: Menu button (reset base)
  - 8: Manufacturer button (base forward)
- Additional buttons can be mapped as needed.
"""

class ControlMode(Enum):
    BASE = 0
    ARM = 1

class JoystickController(Node):
    def __init__(self):
        super().__init__("joystick_controller")
        self.declare_parameter("use_sim", True)
        self.use_sim = self.get_parameter("use_sim").get_parameter_value().bool_value
        self.declare_parameter("control_frequency", 2.0)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        if self.use_sim:
            self.base_pub = self.create_publisher(
                Float64MultiArray, "/tidybot_base_vel_controller/commands", 10
            )
        else:
            raise NotImplementedError("Non-simulation mode is not implemented yet.")

        self.arm_pub = self.create_publisher(
            TwistStamped, "/tidybot/arm/twist", 10
        )
        self.gripper_pub = self.create_publisher(
            Float64, "/tidybot/gripper/command", 10
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

        self.control_enabled = False
        self.control_mode = None

    def joy_callback(self, msg: Joy):
        if self.cmd_queue.full():
            self.cmd_queue.get()
        if (msg.buttons[4] != 0 or msg.buttons[5] != 0) and not self.control_enabled:
            self.control_enabled = True
            self.get_logger().info("Joystick control enabled")
        elif msg.buttons[4] == 0 and msg.buttons[5] == 0 and self.control_enabled:
            self.control_enabled = False
            self.get_logger().info("Joystick control disabled")
        if msg.buttons[0] == 1 and msg.buttons[1] == 0 and self.control_mode != ControlMode.ARM:
            self.control_mode = ControlMode.ARM
            self.get_logger().info("Switch to ARM mode")
        elif msg.buttons[1] == 1 and msg.buttons[0] == 0 and self.control_mode != ControlMode.BASE:
            self.control_mode = ControlMode.BASE
            self.get_logger().info("Switch to BASE mode")
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
        arm_cmd.header.frame_id = "end_effector_link"  # Adjust to your robot's base
        arm_cmd.twist.linear.x = 0.0
        arm_cmd.twist.linear.y = 0.0
        arm_cmd.twist.linear.z = 0.0
        arm_cmd.twist.angular.x = 0.0
        arm_cmd.twist.angular.y = 0.0
        arm_cmd.twist.angular.z = 0.0
        # if the command queue is not empty and control is enabled
        if not self.cmd_queue.empty() and self.control_enabled:
            cmd = self.cmd_queue.get()
            if self.control_mode == ControlMode.BASE:
                # Base control (scale x and y by 0.5 for sensitivity)
                base_cmd_raw = np.array([cmd.axes[1] * 0.5, cmd.axes[0] * 0.5, cmd.axes[3]])
                # get transform from world to base
                try:
                    base_tf = self.tf_buffer.lookup_transform(
                        "world", "base", Time()
                    )
                    # transform the joystick input to base frame
                    base_cmd.data = np.dot(
                    R.from_quat(
                        [
                            base_tf.transform.rotation.x,
                            base_tf.transform.rotation.y,
                            base_tf.transform.rotation.z,
                            base_tf.transform.rotation.w,
                        ]
                    ).as_matrix(),
                    base_cmd_raw,
                )
                except tf2_ros.LookupException as e:
                    self.get_logger().warning(
                        f"Failed to get transform: {e}. Skipping base control."
                    )
            elif self.control_mode == ControlMode.ARM:
                arm_cmd.twist.linear.x = cmd.axes[0] * 0.05
                arm_cmd.twist.linear.y = cmd.axes[1] * 0.05
                arm_cmd.twist.linear.z = cmd.axes[6] * 0.05
                arm_cmd.twist.angular.x = cmd.axes[3] * 0.05
                arm_cmd.twist.angular.y = cmd.axes[4] * 0.05
                arm_cmd.twist.angular.z = cmd.axes[7] * 0.05
                self.get_logger().info(
                    f"Arm command: linear({arm_cmd.twist.linear.x}, {arm_cmd.twist.linear.y}, {arm_cmd.twist.linear.z}), "
                    f"angular({arm_cmd.twist.angular.x}, {arm_cmd.twist.angular.y}, {arm_cmd.twist.angular.z})"
                )

        self.base_pub.publish(base_cmd)
        self.arm_pub.publish(arm_cmd)

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
