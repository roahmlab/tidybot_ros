import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import TwistStamped
# ServoCommandType import removed - no longer needed with C++ API
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
import tf2_ros
from tf2_ros import TransformBroadcaster
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.time import Time

import gc
import threading
from enum import Enum

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

        axis_mapping_defaults = (
            ("base_linear_x", 1),
            ("base_linear_y", 0),
            ("base_angular_z", 3),
            ("arm_linear_x", 0),
            ("arm_linear_y", 1),
            ("arm_linear_z", 7),
            ("arm_angular_x", 4),
            ("arm_angular_y", 3),
            ("arm_roll_positive", 2),
            ("arm_roll_negative", 5),
            ("gripper_horizontal", 6),
        )
        axis_scale_defaults = (
            ("base_linear_x", 0.5),
            ("base_linear_y", 0.5),
            ("base_angular_z", 0.5),
            ("arm_linear_x", 1.0),
            ("arm_linear_y", -1.0),
            ("arm_linear_z", -1.0),
            ("arm_angular_x", -1.0),
            ("arm_angular_y", -1.0),
            ("arm_roll_positive", 1.0),
            ("arm_roll_negative", 1.0),
            ("gripper_horizontal", 1.0),
        )
        button_mapping_defaults = (
            ("arm_mode", 0),
            ("base_mode", 1),
            ("frame_base", 2),
            ("frame_ee", 3),
            ("enable_normal", 4),
            ("enable_turbo", 5),
            ("reset_env", 8),
        )

        self.declare_parameters("axis_mapping", axis_mapping_defaults)
        self.declare_parameters("axis_scale", axis_scale_defaults)
        self.declare_parameters("button_mapping", button_mapping_defaults)

        self._axis_warnings = set()
        self._axis_scale_warnings = set()
        self._button_warnings = set()

        self.axis_map = self._load_mapping("axis_mapping", axis_mapping_defaults, int, self._axis_warnings)
        self.axis_scale = self._load_mapping("axis_scale", axis_scale_defaults, float, self._axis_scale_warnings)
        self.button_map = self._load_mapping("button_mapping", button_mapping_defaults, int, self._button_warnings)

        self.control_enabled = False
        self.control_mode = None
        self.turbo_mode = False
        self.arm_control_frame = ArmControlFrame.EE

        self.gripper_state = 0.0  # initial gripper state (opened)
        self.last_control_enabled = False
        self.reset_button_pressed = False
        self.last_reset_time = None

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        
        self.arm_pub = self.create_publisher(
            TwistStamped, "/tidybot/arm/target_vel", 10
        )
        self.gripper_pub = self.create_publisher(
            Float64, "/tidybot/gripper/commands", 10
        )
        self.base_pub = self.create_publisher(Float64MultiArray, "/tidybot/base/target_vel", 10)
        if self.use_sim:
            self.base_pub_sim = self.create_publisher(
                Float64MultiArray, "/tidybot_base_vel_controller/commands", 10
            )

        self.reset_arm_cli = self.create_client(Empty, "/tidybot/hardware/arm/reset")
        self.reset_base_cli = self.create_client(Empty, "/tidybot/hardware/base/reset")

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

        self.cmd_lock = threading.Lock()
        self.latest_cmd = None
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def joy_callback(self, msg: Joy):
        with self.cmd_lock:
            self.latest_cmd = msg
        normal_pressed = self._button_pressed(msg, "enable_normal")
        turbo_pressed = self._button_pressed(msg, "enable_turbo")
        hardware_enabled = normal_pressed or turbo_pressed
        self.turbo_mode = turbo_pressed
        if hardware_enabled and not self.control_enabled:
            self.control_enabled = True
            if self.turbo_mode:
                self.get_logger().info(f"{GREEN}Joystick control enabled: TURBO mode{RESET}")
            else:
                self.get_logger().info(f"{GREEN}Joystick control enabled: NORMAL mode{RESET}")
        elif not hardware_enabled and self.control_enabled:
            self.control_enabled = False
            self.get_logger().info(f"{RED}Joystick control disabled{RESET}")
        if self._button_pressed(msg, "arm_mode") and not self._button_pressed(msg, "base_mode") and self.control_mode != ControlMode.ARM:
            self.control_mode = ControlMode.ARM
            self.get_logger().info(f"{GREEN}Switch to ARM mode{RESET}")
        elif self._button_pressed(msg, "base_mode") and not self._button_pressed(msg, "arm_mode") and self.control_mode != ControlMode.BASE:
            self.control_mode = ControlMode.BASE
            self.get_logger().info(f"{GREEN}Switch to BASE mode{RESET}")
        if self._button_pressed(msg, "frame_base") and not self._button_pressed(msg, "frame_ee") and self.arm_control_frame != ArmControlFrame.BASE:
            self.arm_control_frame = ArmControlFrame.BASE
            self.get_logger().info(f"{GREEN}Switch to ARM BASE frame{RESET}")
        elif self._button_pressed(msg, "frame_ee") and not self._button_pressed(msg, "frame_base") and self.arm_control_frame != ArmControlFrame.EE:
            self.arm_control_frame = ArmControlFrame.EE
            self.get_logger().info(f"{GREEN}Switch to ARM END-EFFECTOR frame{RESET}")

        reset_pressed = self._button_pressed(msg, "reset_env")
        if reset_pressed and not self.reset_button_pressed:
            self.handle_reset_request()
        self.reset_button_pressed = reset_pressed

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
            return
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
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
                if self.use_sim:
                    self.base_pub_sim.publish(base_cmd)
                self.arm_pub.publish(arm_cmd)
                self.publish_gripper()
            self.last_control_enabled = False
            return

        self.last_control_enabled = True
        with self.cmd_lock:
            cmd = self.latest_cmd

        if self.control_mode == ControlMode.BASE:
            scale = 2.0 if self.turbo_mode else 1.0
            base_values = [
                self._get_axis(cmd, "base_linear_x", multiplier=scale),
                self._get_axis(cmd, "base_linear_y", multiplier=scale),
                self._get_axis(cmd, "base_angular_z", multiplier=scale),
            ]
            base_cmd_msg = Float64MultiArray()
            base_cmd_msg.data = base_values
            self.base_pub.publish(base_cmd_msg)
            if self.use_sim:
                self.base_pub_sim.publish(base_cmd_msg)

        elif self.control_mode == ControlMode.ARM:
            multiplier = 2.0 if self.turbo_mode else 1.0
            arm_cmd.twist.linear.x = self._get_axis(cmd, "arm_linear_x", multiplier=multiplier)
            arm_cmd.twist.linear.y = self._get_axis(cmd, "arm_linear_y", multiplier=multiplier)
            arm_cmd.twist.linear.z = self._get_axis(cmd, "arm_linear_z", multiplier=multiplier)
            arm_cmd.twist.angular.x = self._get_axis(cmd, "arm_angular_x", multiplier=multiplier)
            arm_cmd.twist.angular.y = self._get_axis(cmd, "arm_angular_y", multiplier=multiplier)
            arm_cmd.twist.angular.z = self._compute_arm_roll(cmd, multiplier)

            # Gripper control
            if cmd:
                gripper_axis = self._get_axis(cmd, "gripper_horizontal", multiplier=multiplier)
                if gripper_axis > 0.1:  # open
                    self.gripper_state = min(self.gripper_state + 0.02, 0.81)
                elif gripper_axis < -0.1:  # close
                    self.gripper_state = max(self.gripper_state - 0.02, 0.0)
            self.arm_pub.publish(arm_cmd)
            self.publish_gripper()

    def publish_gripper(self):
        msg = Float64()
        msg.data = float(self.gripper_state)
        self.gripper_pub.publish(msg)

    def handle_reset_request(self):
        now = self.get_clock().now()
        if self.last_reset_time is not None:
            elapsed_ns = (now - self.last_reset_time).nanoseconds
            if elapsed_ns < 5_000_000_000:
                remaining = (5_000_000_000 - elapsed_ns) / 1e9
                self.get_logger().warn(
                    f"{YELLOW}Reset ignored; wait {remaining:.1f}s before resetting again{RESET}"
                )
                return

        self.get_logger().info(f"{GREEN}Requesting arm and base reset...{RESET}")
        if not self.use_sim:
            self._call_reset_service(self.reset_arm_cli, "arm")
            self._call_reset_service(self.reset_base_cli, "base")
        self.last_reset_time = now

    def _call_reset_service(self, client, label):
        if not client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(f"{YELLOW}{label.capitalize()} reset service unavailable{RESET}")
            return

        future = client.call_async(Empty.Request())

        def _done(fut, component=label):
            try:
                fut.result()
                self.get_logger().info(f"{GREEN}{component.capitalize()} reset succeeded{RESET}")
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"{RED}{component.capitalize()} reset failed: {exc}{RESET}")

        future.add_done_callback(_done)

    def _get_axis(self, msg : Joy, key, default=0.0, multiplier=1.0) -> float:
        if msg is None:
            return default
        if key not in self.axis_map:
            self._warn_once(self._axis_warnings, key, f"Axis mapping '{key}' not provided; using {default}")
            return default
        idx = self.axis_map[key]
        if idx >= len(msg.axes):
            self._warn_once(
                self._axis_warnings,
                key,
                f"Axis index {idx} for '{key}' out of range (len={len(msg.axes)}); using {default}",
            )
            return default
        if not self.control_enabled:
            return default
        value = msg.axes[idx]
        scale = self.axis_scale.get(key, 1.0)
        return value * scale * multiplier

    def _get_axis_raw(self, msg : Joy, key, default=0.0) -> float:
        if msg is None:
            return default
        if key not in self.axis_map:
            self._warn_once(self._axis_warnings, key, f"Axis mapping '{key}' not provided; using {default}")
            return default
        idx = self.axis_map[key]
        if idx >= len(msg.axes):
            self._warn_once(
                self._axis_warnings,
                key,
                f"Axis index {idx} for '{key}' out of range (len={len(msg.axes)}); using {default}",
            )
            return default
        return msg.axes[idx]

    def _button_pressed(self, msg : Joy, key) -> bool:
        if msg is None:
            return False
        if key not in self.button_map:
            self._warn_once(self._button_warnings, key, f"Button mapping '{key}' not provided; treating as not pressed")
            return False
        idx = self.button_map[key]
        if idx >= len(msg.buttons):
            self._warn_once(
                self._button_warnings,
                key,
                f"Button index {idx} for '{key}' out of range (len={len(msg.buttons)}); treating as not pressed",
            )
            return False
        return msg.buttons[idx] != 0

    def _compute_arm_roll(self, msg, multiplier=1.0):
        if msg is None or not self.control_enabled:
            return 0.0
        pos_raw = self._get_axis_raw(msg, "arm_roll_positive", default=1.0)
        neg_raw = self._get_axis_raw(msg, "arm_roll_negative", default=1.0)
        pos_delta = (pos_raw - 1.0) * self.axis_scale.get("arm_roll_positive", 1.0)
        neg_delta = (neg_raw - 1.0) * self.axis_scale.get("arm_roll_negative", 1.0)
        roll = (-pos_delta + neg_delta) / 2.0
        return roll * multiplier

    def _load_mapping(self, prefix, defaults, cast_fn, cache):
        mapping = {name: cast_fn(value) for name, value in defaults}
        params = self.get_parameters_by_prefix(prefix)
        for name, parameter in params.items():
            value = parameter.value
            try:
                mapping[name] = cast_fn(value)
            except (TypeError, ValueError):
                fallback = mapping.get(name)
                self._warn_once(
                    cache,
                    name,
                    f"Invalid value '{value}' for {prefix}.{name}; using default {fallback}",
                )
        return mapping

    def _warn_once(self, cache, key, message):
        if key in cache:
            return
        cache.add(key)
        self.get_logger().warn(f"{YELLOW}{message}{RESET}")

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
