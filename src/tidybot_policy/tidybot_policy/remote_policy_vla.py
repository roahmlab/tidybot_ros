#! /home/yuandi/miniforge3/envs/openvla/bin/python
from transformers import AutoModelForVision2Seq, AutoProcessor
from torch.ao.quantization import quantize_dynamic
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Float64MultiArray, Header
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from peft import PeftModel, PeftConfig
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import torch
import cv2
import json
import threading
import time
import os

class OpenVLANode(Node):
    def __init__(self):
        super().__init__('openvla_node')

        # Declare ROS parameters
        self.declare_parameter('merged_ckpt', '')
        self.declare_parameter('adapter_ckpt', '')
        self.declare_parameter('instruction', 'pick up the object')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_action = np.zeros(7)
        self.action_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to selected camera feed
        self.visual_sub = self.create_subscription(
            CompressedImage,
            '/tidybot/camera_ext/color/compressed',
            self.image_callback,
            qos
        )
        # Publisher of inferred action
        self.action_pub = self.create_publisher(Float64MultiArray, '/tidybot/arm/delta_commands', 10)

        # Resolve checkpoint paths from ROS parameters
        merged_ckpt = self.get_parameter('merged_ckpt').get_parameter_value().string_value
        adapter_ckpt = self.get_parameter('adapter_ckpt').get_parameter_value().string_value

        if not merged_ckpt and not adapter_ckpt:
            self.get_logger().fatal('Must provide either merged_ckpt or adapter_ckpt')
            raise RuntimeError('No checkpoint path provided')

        if merged_ckpt:
            if adapter_ckpt:
                self.get_logger().warn('Both merged_ckpt and adapter_ckpt provided; ignoring adapter')
            # Load fully merged checkpoint directly
            self.get_logger().info(f'Loading merged checkpoint from {merged_ckpt}')
            self.processor = AutoProcessor.from_pretrained(merged_ckpt, trust_remote_code=True)
            self.vla = AutoModelForVision2Seq.from_pretrained(
                merged_ckpt,
                torch_dtype=torch.bfloat16,
                low_cpu_mem_usage=True,
                trust_remote_code=True
            ).to("cuda:0")
            ckpt_dir = merged_ckpt
        else:
            # Load base model + LoRA adapter
            self.get_logger().info(f'Loading base model + adapter from {adapter_ckpt}')
            self.processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
            base_model = AutoModelForVision2Seq.from_pretrained(
                "openvla/openvla-7b",
                torch_dtype=torch.bfloat16,
                low_cpu_mem_usage=True,
                trust_remote_code=True
            ).to("cuda:0")
            self.vla = PeftModel.from_pretrained(
                base_model,
                adapter_ckpt,
            ).to("cuda:0")
            ckpt_dir = adapter_ckpt

        # Load dataset statistics from checkpoint directory
        stats_path = os.path.join(ckpt_dir, 'dataset_statistics.json')
        if os.path.exists(stats_path):
            with open(stats_path, 'r') as f:
                stats = json.load(f)
            # The JSON may be wrapped as {"tidybot_vla": {"action": ...}} or just {"action": ...}
            if 'tidybot_vla' in stats:
                stats = stats['tidybot_vla']
            self.vla.config.norm_stats['tidybot_vla'] = stats
            self.get_logger().info(f'Loaded dataset statistics from {stats_path}')
        else:
            self.get_logger().warn(f'No dataset_statistics.json found at {stats_path}')

        # Start threads AFTER model is loaded
        self._inference_thread_running = True
        self.inference_thread = threading.Thread(target=self.inference_loop)
        self.inference_thread.daemon = True
        self.inference_thread.start()

        self._publishing_thread_running = True
        self.publisher_thread = threading.Thread(target=self.publish_loop)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

        self.get_logger().info('Model loaded and threads started')

    def image_callback(self, msg: CompressedImage):
        # Store latest image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR by default
        self.latest_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f"Received image")

        # DEBUG: save image once per second to check for corruption
        now = time.time()
        if not hasattr(self, '_last_save_time') or now - self._last_save_time >= 1.0:
            self._last_save_time = now
            debug_dir = '/tmp/vla_debug'
            os.makedirs(debug_dir, exist_ok=True)
            fname = os.path.join(debug_dir, f'{int(now)}.png')
            cv2.imwrite(fname, image)
            self.get_logger().info(f'DEBUG: saved image to {fname}')

    def inference_loop(self):
        rate_hz = 5.0
        interval = 1.0 / rate_hz
        while self._inference_thread_running and rclpy.ok():
            try:
                if self.latest_image is None:
                    time.sleep(0.1)
                    continue

                self.get_logger().info(f"Running inference")

                # Convert latest image to PIL
                pil_image = PILImage.fromarray(self.latest_image)
                
                # Prepare input prompt
                instruction = self.get_parameter('instruction').get_parameter_value().string_value
                prompt = f"In: What action should the robot take to {instruction}?\nOut:"

                # Tokenize input
                inputs = self.processor(prompt, pil_image, return_tensors="pt").to("cuda:0", dtype=torch.bfloat16)

                # Predict action
                with torch.no_grad():
                    action = self.vla.predict_action(**inputs, unnorm_key="tidybot_vla", do_sample=False)
                with self.action_lock:
                    self.latest_action = action

                self.get_logger().info(f'Inferred action: {action}')

                time.sleep(interval)
            except Exception as e:
                self.get_logger().error(f'Inference error: {e}', throttle_duration_sec=5.0)
                import traceback
                self.get_logger().error(traceback.format_exc(), throttle_duration_sec=5.0)
                time.sleep(1.0)

    def publish_loop(self):
        rate_hz = 15.0
        interval = 1.0 / rate_hz
        while self._publishing_thread_running and rclpy.ok():
            with self.action_lock:
                action_copy = self.latest_action.copy()

            msg = Float64MultiArray()
            msg.data = action_copy.tolist()
            self.action_pub.publish(msg)

            time.sleep(interval)

def main():
    rclpy.init()
    node = OpenVLANode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
