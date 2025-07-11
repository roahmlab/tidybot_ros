import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from ros_gz_interfaces.msg import WorldReset, WorldControl, EntityFactory
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity
from controller_manager_msgs.srv import SwitchController
from ament_index_python.packages import get_package_share_directory
from controller_manager import (
    load_controller,
    configure_controller,
    switch_controllers,
)
import time

robot_description_path = get_package_share_directory("tidybot_description")

class StateController(Node):
    def __init__(self):
        super().__init__("state_controller")
        self.reset_world_cli = self.create_client(ControlWorld, "/world/empty/control")
        self.spawn_tidybot_cli = self.create_client(SpawnEntity, "/world/empty/create")
        self.reset_tf_buffer_cli = self.create_client(
            Empty, "/reset_tf_buffer"
        )
        self.reset_time_cli = self.create_client(
            Empty, "/rviz/reset_time"
        )
        while not self.reset_world_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.reset_world_cli} service to become available..."
            )
        self.get_logger().info(f"Connected to {self.reset_world_cli}")
        while not self.spawn_tidybot_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.spawn_tidybot_cli} service to become available..."
            )
        self.get_logger().info(f"Connected to {self.spawn_tidybot_cli}")
        while not self.reset_tf_buffer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.reset_tf_buffer_cli} service to become available..."
            )
        self.get_logger().info(f"Connected to {self.reset_tf_buffer_cli}")
        while not self.reset_time_cli.wait_for_service(timeout_sec=1.0):    
            self.get_logger().info(
                f"Waiting for {self.reset_time_cli} service to become available..."
            )
        self.get_logger().info(f"Connected to {self.reset_time_cli}")

    def reset_world(self):
        self.get_logger().info("Resetting world...")
        request = ControlWorld.Request()
        control = WorldControl()
        reset = WorldReset()
        reset.all = True
        control.reset = reset
        control.pause = True
        request.world_control = control
        future = self.reset_world_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                res = future.result()
                self.get_logger().info(f"Result: {res}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call did not complete")

    def spawn_tidybot(self):
        self.get_logger().info("Spawning tidybot...")
        request = SpawnEntity.Request()
        entity = EntityFactory()
        entity.name = "tidybot"
        entity.allow_renaming = True
        entity.sdf_filename = robot_description_path + "/urdf/tidybot.urdf"
        request.entity_factory = entity
        future = self.spawn_tidybot_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                res = future.result()
                self.get_logger().info(f"Spawn tidybot result: {res}")
                if res:
                    self.get_logger().info("Spawning ros2 controllers...")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call did not complete")

    def spawn_controllers(self, ctlr_names):
        self.get_logger().info(f"Spawning controllers: {ctlr_names}")
        for name in ctlr_names:
            ret = load_controller(self, "controller_manager", name)
            if not ret.ok:
                self.get_logger().error(f"Failed to load controller {name}")
            else:
                self.get_logger().info(f"Controller {name} loaded successfully")
            ret = configure_controller(self, "controller_manager", name)
            if not ret.ok:
                self.get_logger().error(f"Failed to configure controller {name}")
            else:
                self.get_logger().info(f"Controller {name} configured successfully")
            ret = switch_controllers(
                self,
                "controller_manager",
                [],
                [name],
                strict=SwitchController.Request.STRICT,
                activate_asap=True,
                timeout=5.0,
            )
            if not ret.ok:
                self.get_logger().error(f"Failed to switch controller {name}")
            else:
                self.get_logger().info(f"Controller {name} switched successfully")

    def unpause_world(self):
        self.get_logger().info("Unpausing world...")
        request = ControlWorld.Request()
        control = WorldControl()
        control.pause = False
        request.world_control = control
        future = self.reset_world_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                res = future.result()
                self.get_logger().info(f"World unpaused: {res}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call did not complete")

    def reset_tf_buffer(self):
        self.get_logger().info("Resetting TF buffer...")
        request = Empty.Request()
        future = self.reset_tf_buffer_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                res = future.result()
                self.get_logger().info("TF buffer reset successfully")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call did not complete")

    def reset_time(self):
        self.get_logger().info("Resetting time in RViz...")
        request = Empty.Request()
        future = self.reset_time_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                res = future.result()
                self.get_logger().info("Time reset successfully")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call did not complete")


def main(args=None):
    rclpy.init(args=args)
    node = StateController()
    try:
        # TODO: replace sleep with a more robust wait mechanism
        node.reset_world()
        node.reset_tf_buffer()
        node.reset_time()
        time.sleep(1)  # Wait a moment before spawning
        node.spawn_tidybot()
        time.sleep(5)  # Wait a moment before spawning controllers
        # node.reset_time()
        node.unpause_world()
        node.spawn_controllers(
            [
                "joint_state_broadcaster",
                "tidybot_base_pos_controller",
                "gen3_7dof_controller",
                "gen3_lite_2f_controller",
            ]
        )
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
