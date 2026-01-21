#!/home/yuandi/miniforge3/envs/openpi/bin/python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

import numpy as np
import cv2
cv2.setNumThreads(1)
from cv_bridge import CvBridge
import threading
import time
import os

# --- JAX Configuration
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false" 

import jax
from openpi.training import config as _config
from openpi.policies import policy_config
from openpi.shared import download
from openpi.policies.droid_policy import DroidInputs, DroidOutputs
import openpi.models.model as _model

def convert_tensors_to_numpy(data: dict) -> dict:
    if "image" in data:
        new_images = {}
        image_mask = {} # Initialize the mask dictionary
        
        for k, v in data["image"].items():
            img = np.array(v)
            
            # Fix Shape and Type
            if img.dtype != np.uint8:
                img = (img * 255).astype(np.uint8) if img.max() <= 1.01 else img.astype(np.uint8)
            
            new_images[k] = img
            image_mask[k] = True 
            
        data["image"] = new_images
        data["image_mask"] = image_mask

    if "state" in data:
        data["state"] = np.array(data["state"])
        # Create state mask (True means "this state is valid")
        data["state_mask"] = np.ones(data["state"].shape[0], dtype=bool)

    if "right_wrist_0_rgb" not in data["image"]:
            # Create a black 224x224 image
            data["image"]["right_wrist_0_rgb"] = np.zeros((224, 224, 3), dtype=np.uint8)
            # Set mask to False so the model ignores this "fake" image
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
        self.latest_joint_state = None
        
        # Action Data
        self.action_lock = threading.Lock()
        self.latest_action = np.zeros(8, dtype=np.float32)
        self.last_target_q = None

        # TF Setup for action chunk visualization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = "base" 
        self.ee_frame = "bracelet_link"

        # ----------------------------------------------
        # Model Loading
        # ----------------------------------------------
        self.get_logger().info("Loading JAX model...")
        config = _config.get_config("pi0_tidybot")
        ckpt_dir = "/home/yuandi/openpi/checkpoints/pi0_tidybot/pi0_tidybot_1/9999"
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

        self.arm_pub = self.create_publisher(
            Float64MultiArray, "/tidybot/arm/delta_commands", 10)

        self.gripper_pub = self.create_publisher(
            Float64, "/tidybot/hardware/gripper/commands", 10)

        self.viz_pub = self.create_publisher(
            PoseArray, "/visual_trajectory", viz_qos)

        self._running = True

        # Publisher Thread
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

    def get_current_ee_pose_matrix(self):
        """Helper to get 4x4 matrix of current EE pose from TF"""
        try:
            # Look up transform from base to EE
            t = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.ee_frame, 
                rclpy.time.Time())
            
            # Convert translation to vector
            trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            
            # Convert quaternion to rotation matrix
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            rot_mat = R.from_quat(quat).as_matrix()
            
            # Build 4x4 matrix
            mat = np.eye(4)
            mat[:3, :3] = rot_mat
            mat[:3, 3] = trans
            return mat
        
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}")
            return None
    
    def publish_trajectory_viz(self, start_matrix, action_chunk):
        """
        Generates and publishes a PoseArray from the action chunk.
        """
        pose_array = PoseArray()
        pose_array.header.frame_id = self.base_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        current_matrix = start_matrix.copy()

        for action in action_chunk:
            deltas = action[:6]
            delta_mat = np.eye(4)
            delta_mat[:3, 3] = deltas[:3] # Translation
            
            # Rotation (Euler -> Matrix)
            r_delta = R.from_euler('xyz', deltas[3:6], degrees=False)
            delta_mat[:3, :3] = r_delta.as_matrix()

            # Apply Delta
            current_matrix[:3, 3] += deltas[:3]
            current_matrix[:3, :3] = r_delta.as_matrix() @ current_matrix[:3, :3]

            # Convert back to Pose msg
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
            self.latest_joint_state = msg

    def control_loop(self):
        control_dt = 1.0 / 10.0 

        while self._running and rclpy.ok():
            img_ext, img_wrist, joint_state = None, None, None
            while img_ext is None and self._running:
                with self.data_lock:
                    if self.latest_ext is not None and self.latest_joint_state is not None:
                        img_ext = self.latest_ext
                        img_wrist = self.latest_wrist
                        joint_state = self.latest_joint_state
                if img_ext is None or img_wrist is None or joint_state is None:
                    time.sleep(0.01)

            try:
                # Prepare State
                joint_pos = np.array(joint_state.position[:7], dtype=np.float32)
                gripper = np.array([joint_state.position[7]], dtype=np.float32)
                state = np.concatenate([joint_pos, gripper], axis=0)
                
                start_pose_mat = self.get_current_ee_pose_matrix()
 
                example = {
                    "image": {
                        "base_0_rgb": img_ext,        
                        "left_wrist_0_rgb": img_wrist,
                    },
                    "state": state,                    
                    "prompt": "What action should the robot take to pick up the black cube?",
                }
                example = convert_tensors_to_numpy(example)

                # Inference
                self.get_logger().info("Running inference...")
                out = self.policy.infer(example)
                action_chunk = out["actions"] # Shape: [Chunk, 50]
                
            except Exception as e:
                self.get_logger().error(f"Inference failed: {e}")
                time.sleep(0.1)
                continue

            # Execution Loop
            for action in action_chunk[:15]:
                if not self._running or not rclpy.ok(): break

                # Publish trajectory visual
                if start_pose_mat is not None:
                    self.publish_trajectory_viz(start_pose_mat, action_chunk)

                step_start = time.time()

                # EXTRACT DELTAS
                ee_deltas = action[:6].astype(np.float64)
                
                # EXTRACT GRIPPER
                gripper_val = float(action[6])

                # PUBLISH DELTAS
                delta_msg = Float64MultiArray()
                delta_msg.data = ee_deltas.tolist()
                self.arm_pub.publish(delta_msg)

                # PUBLISH GRIPPER
                gripper_msg = Float64()
                gripper_msg.data = gripper_val
                self.gripper_pub.publish(gripper_msg)

                # Maintain control frequency
                elapsed = time.time() - step_start
                time.sleep(max(0.0, control_dt - elapsed))
            
            time.sleep(0.5)

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
