#!/usr/bin/env python3
"""
Isaac Lab Bridge Node

Bridges TidyBot's ROS 2 control interfaces to Isaac Lab simulation.
This node runs as a standard ROS 2 node and communicates with the
Isaac Lab simulation script via ROS 2 topics.

This is the ROS 2 side of the bridge - the Isaac Lab script publishes
/joint_states and /clock, and subscribes to joint commands.

Subscribed Topics:
    /joint_states (JointState) - Current robot joint states from Isaac Lab

Published Topics:
    (none - commands are published by other nodes like multi_stage_planner)

This node primarily handles:
    - Forwarding /joint_states to robot_state_publisher
    - Managing simulation time synchronization

Usage:
    ros2 launch tidybot_description launch_isaac_lab.launch.py

Note: Unlike isaac_sim_bridge.py, this node does NOT translate controller
topics to a /joint_command topic. Isaac Lab handles joint control directly
through its action space. The drawer_policy and multi_stage_planner publish
directly to topics that Isaac Lab subscribes to.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


class IsaacLabBridge(Node):
    """Bridge node for Isaac Lab integration.
    
    This node monitors the connection to Isaac Lab and provides
    status information. The actual joint state forwarding is handled
    by Isaac Lab publishing directly to /joint_states.
    """
    
    def __init__(self):
        super().__init__('isaac_lab_bridge')
        
        # Parameters
        self.declare_parameter('timeout_sec', 10.0)
        self.timeout_sec = self.get_parameter('timeout_sec').value
        
        # QoS for best-effort sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Monitor joint states from Isaac Lab
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Monitor simulation clock
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            sensor_qos
        )
        
        # State tracking
        self.last_joint_state_time = None
        self.last_clock_time = None
        self.connected = False
        
        # Connection check timer
        self.check_timer = self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info("Isaac Lab Bridge initialized")
        self.get_logger().info("Waiting for Isaac Lab simulation to publish /joint_states and /clock...")
    
    def joint_state_callback(self, msg: JointState):
        """Track joint state reception."""
        self.last_joint_state_time = self.get_clock().now()
        
        if not self.connected:
            self.connected = True
            self.get_logger().info(f"Connected to Isaac Lab! Receiving {len(msg.name)} joints")
    
    def clock_callback(self, msg: Clock):
        """Track clock reception."""
        self.last_clock_time = self.get_clock().now()
    
    def check_connection(self):
        """Periodic connection status check."""
        now = self.get_clock().now()
        
        if self.last_joint_state_time is not None:
            elapsed = (now - self.last_joint_state_time).nanoseconds / 1e9
            if elapsed > self.timeout_sec:
                if self.connected:
                    self.get_logger().warn(f"Lost connection to Isaac Lab (no /joint_states for {elapsed:.1f}s)")
                    self.connected = False
        else:
            self.get_logger().debug("Waiting for /joint_states from Isaac Lab...")


def main(args=None):
    rclpy.init(args=args)
    node = IsaacLabBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
