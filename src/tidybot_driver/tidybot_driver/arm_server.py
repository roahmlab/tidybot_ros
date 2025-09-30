import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Header
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Duration
import queue
import time
import math
from multiprocessing.managers import BaseManager as MPBaseManager
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tidybot_driver.arm_controller import JointCompliantController
from tidybot_driver.constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from tidybot_driver.ik_solver import IKSolver
from tidybot_driver.kinova import TorqueControlledArm
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
import os

CONTROL_FREQ = 20                    # 20 Hz
CONTROL_PERIOD = 1.0 / CONTROL_FREQ  # 50 ms

class ArmServer(Node):
    def __init__(self):
        super().__init__('tidybot_arm_server')
        self.arm = Arm()
        self.get_logger().info('Tidybot arm server started')
        self.arm.reset()

        # Create a subscription to the command topic
        self.arm_cmd_sub = self.create_subscription(
            JointState,
            '/tidybot/physical_arm/commands',
            self.arm_cmd_callback,
            10
        )
        self.arm_delta_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/tidybot/physical_arm/delta_commands', # Exclusively for use in VLA
            self.delta_ee_cmd_callback,
            10
        )
        self.gripper_cmd_sub = self.create_subscription(
            Float64,
            '/tidybot/gripper/commands',
            self.gripper_cmd_callback,
            10
        )

        # Create reset service
        self.reset_srv = self.create_service(
            Empty,
            '/tidybot/physical_arm/reset',
            self.handle_reset_request
        )

        self.target_gripper_pos = 0.0
        self.target_joint_state = np.zeros(7, dtype=np.float64)
        self.arm_state = self.arm.get_state() # Used in delta EE commands
        self.clock = self.get_clock()
        self.jsp_timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        # Arm joint states (joint_1 to joint_7) + Gripper state
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.joint_bounds = {
            'joint_1': None,                  # continuous
            'joint_2': (-2.240000,  2.240000),
            'joint_3': None,                  # continuous
            'joint_4': (-2.570000,  2.570000),
            'joint_5': None,                  # continuous
            'joint_6': (-2.090000,  2.090000),
            'joint_7': None,                  # continuous
        }

    def handle_reset_request(self, request, response):
        self.get_logger().info("Resetting arm")
        self.arm.reset()
        self.get_logger().info("Arm reset complete")
        return response

    def arm_cmd_callback(self, msg):
        # Extract in order of joint_states topic
        joint_order = [f"joint_{i}" for i in range(1, 8)]
        joint_dict = dict(zip(msg.name, msg.position))

        target_joint_state = np.array([joint_dict[j] for j in joint_order], dtype=np.float64)
        self.get_logger().info(f'Received command: {np.concatenate([target_joint_state, [self.arm.arm.gripper_pos]]).tolist()}')

        # Put the command in the queue for the controller to pick up
        self.arm.execute_action(np.concatenate([target_joint_state, [self.target_gripper_pos]]).tolist())\

    def delta_ee_cmd_callback(self, msg):
        # Position deltas
        self.arm_state['arm_pos'] += np.array(msg.data[0:3])

        # Orientation: apply delta RPY to current target quaternion
        delta_rpy = msg.data[3:6]
        current_rot = R.from_quat(self.arm_state['arm_quat'])
        delta_rot = R.from_euler('xyz', delta_rpy)
        new_rot = current_rot * delta_rot  # local frame
        self.arm_state['arm_quat'] = new_rot.as_quat()
        self.get_logger().info(f"Target pose: {self.arm_state['arm_pos']}")

        qpos = self.ik_solver.solve(self.arm_state['arm_pos'], self.arm_state['arm_quat'], self.arm.q)
        self.arm.execute_action(np.concatenate([qpos, [msg[6]]]).tolist())
        
    def gripper_cmd_callback(self, msg):
        # Buffer the target gripper position
        self.target_gripper_pos = msg.data

    def publish_joint_states(self): 
        # Send back current state as feedback
        joint_states = list(self.arm.arm.q)
        bounded_joint_states = []

        for i, name in enumerate(self.joint_bounds.keys()):
            bounds = self.joint_bounds[name]
            angle = joint_states[i]
            if bounds is None:
                # continuous joint → wrap to [-pi, pi]
                angle = (angle + math.pi) % (2 * math.pi) - math.pi
            else:
                # limited joint → wrap to [min, max] by adding/subtracting 2pi
                angle = self.wrap_to_bounds(angle, *bounds)
            bounded_joint_states.append(angle)

        msg = JointState()
        msg.header.stamp = self.clock.now().to_msg()
        msg.name = list(self.joint_bounds.keys()) + ['left_outer_knuckle_joint']
        msg.position = bounded_joint_states + [0.81 * self.arm.arm.gripper_pos]
        # Extend base joints with fixed values
        msg.name.extend(['joint_x', 'joint_y', 'joint_th'])
        msg.position.extend([0.0, 0.0, 0.0])
        self.joint_state_pub.publish(msg)

    def wrap_to_bounds(self, angle: float, min_val: float, max_val: float) -> float:
        """
        Wrap a joint angle to lie within [min_val, max_val] by adding/subtracting 2*pi.
        """
        while angle < min_val:
            angle += 2 * math.pi
        while angle > max_val:
            angle -= 2 * math.pi
        # If wrapping overshoots by multiple revolutions, do modulo
        if angle < min_val or angle > max_val:
            angle = ((angle - min_val) % (2 * math.pi)) + min_val
        return angle

class Arm:
    def __init__(self):
        self.arm = TorqueControlledArm()
        self.arm.set_joint_limits(speed_limits=(7 * (30,)), acceleration_limits=(7 * (80,)))
        self.command_queue = queue.Queue(1)
        self.controller = None
        self.ik_solver = IKSolver(ee_offset=0.12)

    def reset(self):
        # Stop low-level control
        if self.arm.cyclic_running:
            time.sleep(0.75)  # Wait for arm to stop moving
            self.arm.stop_cyclic()

        # Clear faults
        self.arm.clear_faults()

        # Reset arm configuration
        self.arm.open_gripper()
        self.arm.retract()

        # Create new instance of controller
        self.controller = JointCompliantController(self.command_queue)

        # Start low-level control
        self.arm.init_cyclic(self.controller.control_callback)
        while not self.arm.cyclic_running:
            time.sleep(0.01)
    
    def execute_action(self, action):
        # Action corresponds to target joint + gripper state
        self.command_queue.put((action[0:7], action[7]))

    def get_state(self):
        arm_pos, arm_quat = self.arm.get_tool_pose()
        if arm_quat[3] < 0.0:  # Enforce quaternion uniqueness
            np.negative(arm_quat, out=arm_quat)
        state = {
            'arm_pos': arm_pos,
            'arm_quat': arm_quat,
            'gripper_pos': np.array([self.arm.gripper_pos]),
        }
        return state

    def close(self):
        if self.arm.cyclic_running:
            time.sleep(0.75)  # Wait for arm to stop moving
            self.arm.stop_cyclic()
        self.arm.disconnect()

class ArmManager(MPBaseManager):
    pass

ArmManager.register('Arm', Arm)

def main(args=None):
    # manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    # server = manager.get_server()
    # print(f'Arm manager server started at {ARM_RPC_HOST}:{ARM_RPC_PORT}')
    # server.serve_forever()

    rclpy.init(args=args)
    arm_server = ArmServer()
    try:
        # os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO)))
        rclpy.spin(arm_server)
    # except PermissionError:
    #     print('Failed to set real-time scheduling policy, please edit /etc/security/limits.d/99-realtime.conf')
    #     pass
    except KeyboardInterrupt:
        pass
    finally:
        arm_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    # import numpy as np
    # from constants import POLICY_CONTROL_PERIOD
    # manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    # manager.connect()
    # arm = manager.Arm()
    # try:
    #     arm.reset()
    #     for i in range(50):
    #         arm.execute_action({
    #             'arm_pos': np.array([0.135, 0.002, 0.211]),
    #             'arm_quat': np.array([0.706, 0.707, 0.029, 0.029]),
    #             'gripper_pos': np.zeros(1),
    #         })
    #         print(arm.get_state())
    #         time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
    # finally:
    #     arm.close()