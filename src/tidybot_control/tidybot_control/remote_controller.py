import rclpy
import rclpy.exceptions
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from tidybot_utils.msg import WSMsg
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from geometry_msgs.msg import Vector3, TransformStamped

import zmq
import cv2
import gc
from tf_transformations import quaternion_multiply, euler_from_quaternion
import numpy as np

REMOTE_CONTROL_FREQUENCY = 20  # Hz

class RemoteController(Node):
    def __init__(self):
        super().__init__("remote_controller")
        # TODO: add real robot control
        self.declare_parameter("use_sim", True)
        self.use_sim = self.get_parameter("use_sim").get_parameter_value().bool_value
        self.get_logger().info(
            "Remote controller initialized with use_sim: {}".format(self.use_sim)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.reset_service = self.create_service(
            Empty, "/remote_controller/reset", self.reset_env_callback
        )

        self.control_timer = self.create_timer(
            1.0 / REMOTE_CONTROL_FREQUENCY, self.control_loop
        )

        self.joints_observer = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )
        self.last_joint_state = None
        self.base_image_listener = self.create_subscription(
            Image, "/base_camera/image", self.base_image_callback, 10
        )
        self.last_base_image = None
        self.arm_image_listener = self.create_subscription(
            Image, "/arm_camera/image", self.arm_image_callback, 10
        )
        self.last_arm_image = None
        self.bridge = CvBridge()

        self.command_listener = self.create_subscription(
            WSMsg, "/ws_commands", self.command_callback, 10
        )
        self.enabled = False

        self.clock = self.get_clock()
        threshold = JumpThreshold(
            min_forward=None, min_backward=Duration(seconds=-1), on_clock_change=True
        )
        self.jump_handle = self.clock.create_jump_callback(
            threshold, post_callback=self.time_jump_callback
        )

        self.base_offset = TransformStamped()
        self.base_offset.transform.translation.x = 0.12
        self.base_offset.transform.translation.y = 0.0
        self.base_offset.transform.translation.z = 0.374775

        self.base_pub = self.create_publisher(
            Float64MultiArray, "/tidybot_base_pos_controller/commands", 10
        )
        self.arm_pub = self.create_publisher(Pose, "/tidybot/arm/command", 10)
        self.gripper_pub = self.create_publisher(
            Float64, "/tidybot/gripper/command", 10
        )

        # Connection to policy server
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect(f'tcp://localhost:5555')
        self.get_logger().info(f'Connected to policy server at localhost:5555')

    def control_loop(self):
        if not self.enabled:
            return
        pass  
        
        # Get the latest arm pose
        arm_pos, arm_orientation = self.get_arm_pose()
        base_pos, base_orientation = self.get_base_pose()
        _, _, base_yaw = euler_from_quaternion(base_orientation)
        base_image = self.ros_image_to_jpg(self.last_base_image) if self.last_base_image else None
        arm_image = self.ros_image_to_jpg(self.last_arm_image) if self.last_arm_image else None

        obs = {
            "base_pose": np.array([base_pos[0], base_pos[1], base_yaw]),
            "arm_pos": np.array(arm_pos),
            "arm_quat": np.array(arm_orientation),
            "gripper_pos": np.array([self.last_gripper_state]) if self.last_gripper_state is not None else None,
            "base_image": base_image,
            "wrist_image": arm_image,
        }
        # if None in obs:
        #     self.get_logger().warn("Incomplete observation, skipping control loop")
        #     return
        # Send observation to policy server
        req = {'obs': obs}
        self.get_logger().info(f"Sending request to policy server: {req}")
        self.socket.send_pyobj(req)

        rep = self.socket.recv_pyobj()
        self.get_logger().info(f"Received response from policy server: {rep}")

        if rep['action'] is None:
            self.get_logger().warn("Received None actions from policy server, skipping control loop")
            return
        rep_base_pose = rep['action']['base_pose'].tolist()
        rep_arm_pos = rep['action']['arm_pos'].tolist()
        rep_arm_quat = rep['action']['arm_quat'].tolist()
        rep_gripper = rep['action']['gripper_pos'].tolist()

        self.get_logger().info(f"Base pose: {rep_base_pose}, Arm pos: {rep_arm_pos}, Arm quat: {rep_arm_quat}, Gripper: {rep_gripper}")
        # Offset the arm position by the base offset
        try:
            offset = self.tf_buffer.lookup_transform(
                "world", "arm_base_link", Time()
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None, None

        abs_arm_pos = [rep_arm_pos[0] + offset.transform.translation.x,
                       rep_arm_pos[1] + offset.transform.translation.y,
                       rep_arm_pos[2] + offset.transform.translation.z]
        # abs_arm_quat = quaternion_multiply([offset.transform.rotation.w, offset.transform.rotation.x, offset.transform.rotation.y, offset.transform.rotation.z], rep_arm_quat)
        abs_arm_quat = rep_arm_quat
        arm_command = Pose()
        arm_command.position.x = abs_arm_pos[0]
        arm_command.position.y = abs_arm_pos[1]
        arm_command.position.z = abs_arm_pos[2]
        arm_command.orientation.x = abs_arm_quat[0]
        arm_command.orientation.y = abs_arm_quat[1]
        arm_command.orientation.z = abs_arm_quat[2]
        arm_command.orientation.w = abs_arm_quat[3]
        self.arm_pub.publish(arm_command)

        base_command = Float64MultiArray()
        base_command.data = [
            rep_base_pose[0],
            rep_base_pose[1],
            rep_base_pose[2]]
        self.base_pub.publish(base_command)

        if rep_gripper is not None:
            gripper_command = Float64()
            gripper_command.data = rep_gripper[0] / 0.81
            self.gripper_pub.publish(gripper_command)

    def reset_env_callback(self, request, response):
        # Check connection to policy server and reset policy
        default_timeout = self.socket.getsockopt(zmq.RCVTIMEO)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # Temporarily set 1000 ms timeout
        self.socket.send_pyobj({'reset': True})

        try:
            self.socket.recv_pyobj()  # Note: Not secure. Only unpickle data you trust.
        except zmq.error.Again as e:
            raise Exception('Could not communicate with policy server') from e
        self.socket.setsockopt(zmq.RCVTIMEO, default_timeout)  # Put default timeout back

        # Disable policy execution until user presses on screen
        self.enabled = False  # Note: Set to True to run without phone
        return response

    def joint_states_callback(self, msg):
        self.last_gripper_state = msg.position[msg.name.index("left_outer_knuckle_joint")]

    def base_image_callback(self, msg):
        self.last_base_image = msg

    def arm_image_callback(self, msg):
        self.last_arm_image = msg

    def command_callback(self, msg):   
        if msg.teleop_mode:
            self.enabled = True
        else:
            self.enabled = False

    def ros_image_to_jpg(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        # resize the image to to resolution expected by the policy server
        # Note: This is a placeholder, adjust as needed
        v = cv2.resize(cv_image, (84, 84))
        _, jpeg = cv2.imencode('.jpg', v)
        return jpeg
    
    def get_base_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", "base", Time()
            )
            return [transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z], [transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w]
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None, None

    def get_arm_pose(self):
        try:
            # get the relative transform from base to end effector
            transform = self.tf_buffer.lookup_transform(
                "arm_base_link", "end_effector_link", Time()
            )
            return [transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z], [transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w]
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None, None
        

    def time_jump_callback(self, time: Time):
        # re-instantiate the TF listener and buffer to avoid TF_OLD_DATA warning
        self.destroy_subscription(self.tf_listener.tf_sub)
        self.destroy_subscription(self.tf_listener.tf_static_sub)
        del self.tf_listener
        del self.tf_buffer
        gc.collect()
        # Clear both dynamic and static entries
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF buffer reset")

def main(args=None):
    rclpy.init(args=args)
    remote_controller = RemoteController()
    try:
        rclpy.spin(remote_controller)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        remote_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()