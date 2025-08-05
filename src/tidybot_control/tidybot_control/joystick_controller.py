import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped
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

"""Joystick Message mapping:
- Axes:
  - 0: Left stick horizontal (base x)
  - 1: Left stick vertical (base y)
  - 2: Left trigger (1 - -1)
  - 3: Right stick horizontal (base theta)
  - 4: Right stick vertical (y)
  - 5: Right trigger (1 - -1)
  - 6: D-pad horizontal (1 | 0 | -1)
  - 7: D-pad vertical (1 | 0 | -1)
- Buttons:
  - 0: A button (gripper open)
  - 1: B button (gripper close)
  - 2: X button (arm up)
  - 3: Y button (arm down)
  - 4: Left bumper (enable)
  - 5: Right bumper (enable)
  - 6: Overview button (reset arm)
  - 7: Menu button (reset base)
  - 8: Manufacturer button (base forward)
- Additional buttons can be mapped as needed.
"""


class JoystickController(Node):
    def __init__(self):
        super().__init__("joystick_controller")
        self.declare_parameter("use_sim", True)
        self.use_sim = self.get_parameter("use_sim").get_parameter_value().bool_value
        self.declare_parameter("control_frequency", 40.0)
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
            Float64MultiArray, "/tidybot/arm/command", 10
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

    def joy_callback(self, msg: Joy):
        if self.cmd_queue.full():
            self.cmd_queue.get()
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
        base_cmd.data = [0.0, 0.0, 0.0]
        if not self.cmd_queue.empty():
            # check if the control is enabled
            cmd = self.cmd_queue.get()
            if cmd.buttons[4] != 0 or cmd.buttons[5] != 0:
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
        self.base_pub.publish(base_cmd)

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
