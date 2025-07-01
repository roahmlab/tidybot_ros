import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import WorldReset, WorldControl, Entity, EntityFactory
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity
from controller_manager_msgs.srv import SwitchController
from ament_index_python.packages import get_package_share_directory
from controller_manager import (
    load_controller,
    configure_controller,
    switch_controllers,
)
import xacro
from pathlib import Path
import time
import os

robot_description_path = get_package_share_directory("tidybot_description")
doc = xacro.process_file(str(robot_description_path + "/urdf/tidybot.xacro"))
urdf_xml = doc.toxml()
outpath = Path(robot_description_path) / "urdf/tidybot.urdf"
outpath.write_text(urdf_xml, encoding="utf-8")


class StateController(Node):
    def __init__(self):
        super().__init__("state_controller")
        self.reset_service = self.create_client(ControlWorld, "/world/empty/control")
        self.spawn_service = self.create_client(SpawnEntity, "/world/empty/create")
        while not self.reset_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.reset_service} service to become available..."
            )
        self.get_logger().info(f"Connected to {self.reset_service}")
        while not self.spawn_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.spawn_service} service to become available..."
            )
        self.get_logger().info(f"Connected to {self.spawn_service}")

    def reset_and_pause(self):
        self.get_logger().info("Resetting world...")
        request = ControlWorld.Request()
        control = WorldControl()
        reset = WorldReset()
        reset.all = True
        control.reset = reset
        control.pause = True
        request.world_control = control
        future = self.reset_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                res = future.result()
                self.get_logger().info(f"Result: {res}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call did not complete")

    def spawn(self):
        self.get_logger().info("Spawning tidybot...")
        request = SpawnEntity.Request()
        entity = EntityFactory()
        entity.name = "tidybot"
        entity.allow_renaming = True
        entity.sdf_filename = robot_description_path + "/urdf/tidybot.urdf"
        request.entity_factory = entity
        future = self.spawn_service.call_async(request)
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

    def unpause(self):
        self.get_logger().info("Unpausing world...")
        request = ControlWorld.Request()
        control = WorldControl()
        control.pause = False
        request.world_control = control
        future = self.reset_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                res = future.result()
                self.get_logger().info(f"Result: {res}")
                if res:
                    self.get_logger().info("World unpaused successfully")
                else:
                    self.get_logger().error("Failed to unpause world")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call did not complete")


def main(args=None):
    rclpy.init(args=args)
    node = StateController()
    try:
        node.reset_and_pause()
        time.sleep(1)  # Wait a moment before spawning
        node.spawn()
        time.sleep(5)  # Wait a moment before spawning controllers
        node.unpause()
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
