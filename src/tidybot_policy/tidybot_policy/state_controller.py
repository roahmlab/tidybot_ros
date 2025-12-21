from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity
from ament_index_python.packages import get_package_share_directory
import subprocess

robot_description_path = get_package_share_directory("tidybot_description")

RED = "\x1b[31m"
GREEN = "\x1b[32m"
RESET = "\x1b[0m"
BOLD = "\x1b[1m"

class State(Enum):
    IDLE = "idle"
    EPISODE_STARTED = "episode_started"
    EPISODE_FINISHED = "episode_finished"
    ENVIRONMENT_RESET = "environment_reset"


class StateController(Node):
    def __init__(self):
        super().__init__("state_controller")
        
        # Use reentrant callback group for service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # sim_mode: "hardware", "gazebo", or "isaac"
        self.declare_parameter("sim_mode", "hardware")
        self.declare_parameter("use_remote", False)
        self.declare_parameter("record", True)
        self.sim_mode = self.get_parameter("sim_mode").get_parameter_value().string_value
        self.use_remote = self.get_parameter("use_remote").get_parameter_value().bool_value
        self.record = self.get_parameter("record").get_parameter_value().bool_value
        
        self.get_logger().info(f"StateController initialized with sim_mode: {self.sim_mode}")

        self.state_sub = self.create_subscription(
            String, "/teleop_state", self.state_callback, 10
        )

        if self.record:
            self.start_recording_cli = self.create_client(Empty, "/start_recording")
            self.stop_recording_cli = self.create_client(Empty, "/stop_recording")
            self.finalize_recording_cli = self.create_client(SetBool, "/finalize_recording")
            while not self.start_recording_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for start recording service...")
            while not self.stop_recording_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for stop recording service...")
            while not self.finalize_recording_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for finalize recording service...")

        self.reset_arm_cli = self.create_client(Empty, "/tidybot/hardware/arm/reset")
        self.reset_base_cli = self.create_client(Empty, "/tidybot/hardware/base/reset")
        # Teleop controller reset service (resets refs and sends home commands)
        self.reset_controller_cli = self.create_client(Empty, "/tidybot/policy/reset")
        while not self.reset_controller_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for policy reset service...")
        
        # Isaac Sim reset service (provided by isaac_sim_bridge)
        if self.sim_mode == "isaac":
            self.isaac_reset_cli = self.create_client(
                Empty, 
                "/isaac_sim/reset_env",
                callback_group=self.callback_group
            )
            while not self.isaac_reset_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info("Waiting for Isaac Sim reset service...")

        self.state = State.IDLE
        self.awaiting_save_decision = False

    def state_callback(self, msg):
        self.get_logger().info(f"Received state command: {msg.data}")
        match msg.data:
            case "reset_env":
                if self.state == State.IDLE or self.state == State.EPISODE_FINISHED:
                    self.get_logger().info(f"{GREEN}{BOLD}Resetting environment (sim_mode={self.sim_mode})...{RESET}")
                    self.state = State.ENVIRONMENT_RESET
                    # Reset the environment based on sim_mode
                    if self.sim_mode == "gazebo":
                        subprocess.run(["ros2", "run", "tidybot_policy", "reset_env"])
                    elif self.sim_mode == "isaac":
                        # Call Isaac Sim bridge reset service (synchronous call via service)
                        self.reset_isaac_sim()
                    else:  # hardware
                        self.reset_base_cli.call_async(Empty.Request())
                        self.reset_arm_cli.call_async(Empty.Request())
                    # Reset Controller
                    self.reset_controller_cli.call_async(Empty.Request())
            case "episode_started":
                if self.state == State.IDLE or self.state == State.ENVIRONMENT_RESET:
                    self.state = State.EPISODE_STARTED
                    self.get_logger().info(f"{GREEN}{BOLD}Episode started.{RESET}")
                    if self.record:
                        self.start_recording_cli.call_async(Empty.Request())
            case "episode_ended":
                if self.state == State.IDLE or self.state == State.EPISODE_STARTED:
                    self.state = State.EPISODE_FINISHED
                    self.get_logger().info(f"{GREEN}{BOLD}Episode ended.{RESET}")
                    if self.record:
                        # Stop recording first. Save/discard will come via UI socket events.
                        self.stop_recording_cli.call_async(Empty.Request())
                        self.awaiting_save_decision = True
            case "save_episode":
                if self.record and self.state == State.EPISODE_FINISHED and self.awaiting_save_decision:
                    self.finalize_episode(True)
                    self.awaiting_save_decision = False
                    self.get_logger().info(f"{GREEN}{BOLD}Episode saved.{RESET}")
            case "discard_episode":
                if self.record and self.state == State.EPISODE_FINISHED and self.awaiting_save_decision:
                    self.finalize_episode(False)
                    self.awaiting_save_decision = False
                    self.get_logger().info(f"{GREEN}{BOLD}Episode discarded.{RESET}")

    def finalize_episode(self, save: bool):
        if not self.record:
            return
        req = SetBool.Request()
        req.data = save
        self.finalize_recording_cli.call_async(req)

    def reset_isaac_sim(self):
        """Call Isaac Sim bridge reset service and wait for completion."""
        
        self.get_logger().info("Calling Isaac Sim reset service...")
        future = self.isaac_reset_cli.call_async(Empty.Request())
        
        # Wait for the service to complete (with timeout)
        import time
        start_time = time.time()
        timeout = 15.0  # seconds
        
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Timeout ({timeout}s) waiting for Isaac Sim reset")
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Isaac Sim reset complete")


def main(args=None):
    rclpy.init(args=args)
    state_controller = StateController()
    rclpy.spin(state_controller)
    state_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
