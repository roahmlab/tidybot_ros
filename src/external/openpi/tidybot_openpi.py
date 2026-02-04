#!/home/yuandi/miniforge3/envs/openpi/bin/python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

import numpy as np
import cv2
cv2.setNumThreads(1) 
from cv_bridge import CvBridge
import threading
import time
import os

# --- JAX Configuration (Must be before JAX import) ---
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false" 

import jax
from openpi.training import config as _config
from openpi.policies import policy_config

def convert_tensors_to_numpy(data: dict) -> dict:
    if "image" in data:
        new_images = {}
        image_mask = {} 
        
        for k, v in data["image"].items():
            img = np.array(v)
            if img.dtype != np.uint8:
                img = (img * 255).astype(np.uint8) if img.max() <= 1.01 else img.astype(np.uint8)
            new_images[k] = img
            image_mask[k] = True 
            
        data["image"] = new_images
        data["image_mask"] = image_mask

    if "state" in data:
        data["state"] = np.array(data["state"])
        data["state_mask"] = np.ones(data["state"].shape[0], dtype=bool)

    if "right_wrist_0_rgb" not in data["image"]:
            data["image"]["right_wrist_0_rgb"] = np.zeros((224, 224, 3), dtype=np.uint8)
            data["image_mask"]["right_wrist_0_rgb"] = False

    return data

class OpenPiNode(Node):
    def __init__(self):
        super().__init__("openpi_node")

        self.bridge = CvBridge()

        # Shared Data
        self.data_lock = threading.Lock()
        self.latest_ext = None
        self.latest_wrist = None
        self.latest_gripper_state = None 
        
        # Declare a parameter for the initial default prompt
        self.declare_parameter("default_prompt", "Pick up the black cube")
        self.current_prompt = self.get_parameter("default_prompt").value
        self.get_logger().info(f"Initial Prompt: '{self.current_prompt}'")

        # TF Setup 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = "arm_base_link" 
        self.ee_frame = "bracelet_link"

        # ----------------------------------------------
        # Model Loading
        # ----------------------------------------------
        self.get_logger().info("Loading JAX model...")
        config = _config.get_config("pi05_tidybot")
        ckpt_dir = "/home/yuandi/openpi/checkpoints/pi05_tidybot/pi05_tidybot_1/6000"
        self.policy = policy_config.create_trained_policy(config, ckpt_dir)
        
        # ----------------------------------------------
        # ROS Communication
        # ----------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ext_sub = self.create_subscription(
            CompressedImage, "/tidybot/camera_ext/color/compressed",
            self.ext_image_cb, qos)

        self.wrist_sub = self.create_subscription(
            CompressedImage, "/tidybot/camera_wrist/color/compressed",
            self.wrist_image_cb, qos)

        self.joint_sub = self.create_subscription(
            JointState, "/joint_states",
            self.joint_cb, qos)

        self.prompt_sub = self.create_subscription(
            String, "/openpi/prompt",
            self.prompt_cb, 10)

        self.arm_pub = self.create_publisher(
            Float64MultiArray, "/tidybot/arm/delta_commands", 10)

        self.viz_pub = self.create_publisher(
            PoseArray, "/visual_trajectory", viz_qos)

        # ----------------------------------------------
        # Threading
        # ----------------------------------------------
        self._running = True
        self.publisher_thread = threading.Thread(target=self.control_loop)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

        self.get_logger().info("OpenPiNode initialized.")

    def decode_image(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
            if img is not None:
                return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
        return None

    def get_current_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.ee_frame, 
                rclpy.time.Time())
            
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            
            rx = t.transform.rotation.x
            ry = t.transform.rotation.y
            rz = t.transform.rotation.z
            rw = t.transform.rotation.w
            
            rot_mat = R.from_quat([rx, ry, rz, rw]).as_matrix()
            mat = np.eye(4)
            mat[:3, :3] = rot_mat
            mat[:3, 3] = [tx, ty, tz]
            
            pose_vec = [tx, ty, tz, rw, rx, ry, rz]
            
            return mat, pose_vec
        except Exception as e:
            return None, None
    
    def publish_trajectory_viz(self, start_matrix, action_chunk):
        pose_array = PoseArray()
        pose_array.header.frame_id = "arm_base_link"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        current_matrix = start_matrix.copy()

        for action in action_chunk:
            deltas = action[:6]
            r_delta = R.from_euler('xyz', deltas[3:6], degrees=False)
            current_matrix[:3, :3] = r_delta.as_matrix() @ current_matrix[:3, :3]
            current_matrix[:3, 3] += deltas[:3]

            pose = Pose()
            pose.position.x = current_matrix[0, 3]
            pose.position.y = current_matrix[1, 3]
            pose.position.z = current_matrix[2, 3]
            
            q = R.from_matrix(current_matrix[:3, :3]).as_quat()
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            pose_array.poses.append(pose)
            
        self.viz_pub.publish(pose_array)

    # ============================================================
    # Callbacks
    # ============================================================
    def ext_image_cb(self, msg):
        img = self.decode_image(msg)
        if img is not None:
            with self.data_lock:
                self.latest_ext = img

    def wrist_image_cb(self, msg):
        img = self.decode_image(msg)
        if img is not None:
            with self.data_lock:
                self.latest_wrist = img

    def joint_cb(self, msg: JointState):
        with self.data_lock:
            if len(msg.position) > 7:
                self.latest_gripper_state = msg.position[7]

    def prompt_cb(self, msg: String):
        with self.data_lock:
            if msg.data != self.current_prompt:
                self.current_prompt = msg.data
                self.get_logger().info(f"Received new prompt: '{self.current_prompt}'")

    def control_loop(self):
        control_dt = 1.0 / 10.0 

        while self._running and rclpy.ok():
            img_ext, img_wrist, gripper_val = None, None, None
            active_prompt = ""

            # Wait for data and grab the current prompt safely
            while img_ext is None and self._running:
                with self.data_lock:
                    if self.latest_ext is not None and self.latest_gripper_state is not None:
                        img_ext = self.latest_ext
                        img_wrist = self.latest_wrist
                        gripper_val = self.latest_gripper_state
                        active_prompt = self.current_prompt # Capture prompt for this inference cycle
                if img_ext is None or img_wrist is None:
                    time.sleep(0.01)

            start_pose_mat, start_pose_vec = self.get_current_ee_pose()
            
            if start_pose_mat is None:
                self.get_logger().warn("Waiting for TF transform (Base -> EE)...")
                time.sleep(0.5)
                continue

            try:
                state = np.array(start_pose_vec + [gripper_val], dtype=np.float32)
                
                example = {
                    "image": {
                        "base_0_rgb": img_ext,        
                        "left_wrist_0_rgb": img_wrist,
                    },
                    "state": state,                    
                    "prompt": active_prompt
                }
                example = convert_tensors_to_numpy(example)

                out = self.policy.infer(example)
                action_chunk = out["actions"] 
                
            except Exception as e:
                self.get_logger().error(f"Inference failed: {e}")
                time.sleep(0.1)
                continue

            # Execution Loop
            for action in action_chunk[:10]:
                if not self._running or not rclpy.ok(): break
                
                self.publish_trajectory_viz(start_pose_mat, action_chunk)

                step_start = time.time()

                delta_msg = Float64MultiArray()
                delta_msg.data = action[:7].tolist()
                self.arm_pub.publish(delta_msg)

                elapsed = time.time() - step_start
                time.sleep(max(0.0, control_dt - elapsed))
            
            time.sleep(0.15)

def main():
    rclpy.init()
    node = OpenPiNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.publisher_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()