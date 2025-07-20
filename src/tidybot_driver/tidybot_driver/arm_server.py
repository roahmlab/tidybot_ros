import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped

import queue
import time
import math
from multiprocessing.managers import BaseManager as MPBaseManager
import numpy as np
from sensor_msgs.msg import JointState
from tidybot_driver.arm_controller import JointCompliantController
from tidybot_driver.constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from tidybot_driver.ik_solver import IKSolver
from tidybot_driver.kinova import TorqueControlledArm
from tidybot_utils.srv import ResetEnv

class ArmServer(Node):
    def __init__(self):
        super().__init__('tidybot_arm_server')
        self.arm = Arm()
        self.get_logger().info('Tidybot arm server started')
        self.arm.reset()

        # Create a subscription to the command topic
        self.arm_cmd_sub = self.create_subscription(
            Pose,
            '/tidybot/arm/command',
            self.cmd_callback,
            10
        )
        self.gripper_cmd_sub = self.create_subscription(
            Float64,
            '/tidybot/gripper/command',
            self.gripper_cmd_callback,
            10
        )
        self.arm_state_pub = self.create_publisher(
            PoseStamped,
            '/tidybot/arm/pose',
            10
        )
        self.gripper_state_pub = self.create_publisher(
            Float64,
            '/tidybot/gripper/state',
            10
        )
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/tidybot/arm/joint_states',
            10
        )

        # Create reset service
        self.reset_srv = self.create_service(
            ResetEnv,
            '/tidybot/arm/reset',
            self.handle_reset_request
        )

        self.gripper_cmd = Float64()
        self.clock = self.get_clock()
        self.control_timer = self.create_timer(0.05, self.control_callback)

    def handle_reset_request(self, request, response):
        if request.reset:
            self.get_logger().info("Received reset request")
            self.arm.reset()
            response.success = True
        else:
            self.get_logger().warn("Received non-reset request")
            response.success = False

        return response

    def cmd_callback(self, msg):
        target_pos = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
        target_quat = np.array([msg.orientation.x, 
                                msg.orientation.y,
                                msg.orientation.z, 
                                msg.orientation.w], 
                                dtype=np.float64)
        self.get_logger().info(f'Received command: {target_pos}')
        action = {'arm_pos' : target_pos,
                  'arm_quat' : target_quat,
                  'gripper_pos' : np.array(self.gripper_cmd.data, dtype=np.float64)}
        self.arm.execute_action(action)

    def gripper_cmd_callback(self, msg):
        self.gripper_cmd = msg

    def control_callback(self):
        self.arm.arm.update_state()
        pose = PoseStamped()
        pose.header.stamp = self.clock.now().to_msg()
        state = self.arm.get_state()

        # Position
        pose.pose.position.x = float(state['arm_pos'][0])
        pose.pose.position.y = float(state['arm_pos'][1])
        pose.pose.position.z = float(state['arm_pos'][2])

        # Orientation (quaternion)
        pose.pose.orientation.x = float(state['arm_quat'][0])
        pose.pose.orientation.y = float(state['arm_quat'][1])
        pose.pose.orientation.z = float(state['arm_quat'][2])
        pose.pose.orientation.w = float(state['arm_quat'][3])

        self.arm_state_pub.publish(pose)
        gripper_state = Float64()
        gripper_state.data = float(state['gripper_pos'])
        self.gripper_state_pub.publish(gripper_state)

        joint_state = JointState()
        joint_state.header.stamp = self.clock.now().to_msg()
        joint_state.name = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6', 'joint_7',
            'finger_joint'
        ]
        joint_state.position = [angle for angle in self.arm.arm.q] + \
                               [0.8 * self.arm.arm.gripper_pos]
        self.joint_state_pub.publish(joint_state)

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
        qpos = self.ik_solver.solve(action['arm_pos'], action['arm_quat'], self.arm.q)
        self.command_queue.put((qpos, action['gripper_pos'].item()))

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