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
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_action = np.zeros(7)
        self.action_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to camera feed
        self.visual_sub = self.create_subscription(
            CompressedImage,
            '/tidybot/camera_ext/color/compressed',
            self.image_callback,
            qos
        )
        # Publisher of inferred action
        self.action_pub = self.create_publisher(Float64MultiArray, '/tidybot/arm/delta_command', 10)

        # Run inference at fixed 5 Hz, publish action at 15hz
        self._inference_thread_running = True
        self.inference_thread = threading.Thread(target=self.inference_loop)
        self.inference_thread.daemon = True
        self.inference_thread.start()

        self._publishing_thread_running = True
        self.publisher_thread = threading.Thread(target=self.publish_loop)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

        # Load processor and VLA
        self.processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
        base_model = AutoModelForVision2Seq.from_pretrained(
            "openvla/openvla-7b",
            # attn_implementation="flash_attention_2",  
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            trust_remote_code=True
        ).to("cuda:0")

        self.vla = PeftModel.from_pretrained(
            base_model,
            "/home/yuandi/openvla_finetune_adapter_tmp/openvla-7b+kinova_99+b16+lr-0.0005+lora-r32+dropout-0.0",
        ).to("cuda:0")

        with open("/home/yuandi/tensorflow_datasets/dataset_statistics_aef531d2c5a8775d45398cc8cd6fc62f042c82e55b7ade3b35a4c010aa2a24a7.json", "r") as f:
            stats = json.load(f)
        self.vla.config.norm_stats["kinova_99"] = stats

    def image_callback(self, msg: CompressedImage):
        # Store latest image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR by default
        self.latest_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f"Received image")

    def inference_loop(self):
        rate_hz = 5.0
        interval = 1.0 / rate_hz
        while self._inference_thread_running and rclpy.ok():
            if self.latest_image is None:
                time.sleep(0.1)
                continue

            self.get_logger().info(f"Running inference")

            # Convert latest image to PIL
            pil_image = PILImage.fromarray(self.latest_image)
            pil_image.save("/home/yuandi/image.jpg")

            # Prepare input prompt
            prompt = "In: What action should the robot take to pick up the orange?\nOut:"

            # Tokenize input
            inputs = self.processor(prompt, pil_image, return_tensors="pt").to("cuda:0", dtype=torch.bfloat16)

            # Predict action
            with torch.no_grad():
                action = self.vla.predict_action(**inputs, unnorm_key="kinova_99", do_sample=False)
                action[6] = 0.6 if action[6] > 0.2 else 0.0
            with self.action_lock:
                self.latest_action = action

            log_path = os.path.join(os.path.expanduser("~"), "actions_log.txt")
            with open(log_path, "a") as f:
                f.write(" ".join(map(str, action.tolist())) + "\n")

            time.sleep(interval)

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
