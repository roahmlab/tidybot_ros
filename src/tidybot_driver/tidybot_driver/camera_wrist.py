import os
import time
import cv2
import threading
from concurrent.futures import ThreadPoolExecutor, TimeoutError
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
RED = "\x1b[31m"
RESET = "\x1b[0m"
BOLD = "\x1b[1m"

# Listens to camera feeds published by ros2_kortex_vision,
# applies transforms and republishes to tidybot topics
class Camera(Node):
    def __init__(self):
        super().__init__('tidybot_wrist_camera')
        
        # Setup QoS
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parameters
        self.declare_parameter('target_size', 224)
        self.target_size = self.get_parameter('target_size').get_parameter_value().integer_value
        
        # Added default crop_scale parameter since the function requires it
        self.declare_parameter('crop_scale', 1.0)
        self.crop_scale = self.get_parameter('crop_scale').get_parameter_value().double_value
        
        self.br = CvBridge()

        self.pub_color_raw = self.create_publisher(Image, '/tidybot/camera_wrist/color/raw', qos_policy)
        self.pub_color_comp = self.create_publisher(CompressedImage, '/tidybot/camera_wrist/color/compressed', qos_policy)
        self.pub_depth_raw = self.create_publisher(Image, '/tidybot/camera_wrist/depth/raw', qos_policy)
        
        self.sub_color = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.color_callback, 
            qos_policy
        )
        self.sub_depth = self.create_subscription(
            Image, 
            '/camera/depth/image_raw', 
            self.depth_callback, 
            qos_policy
        )
        
        self.get_logger().info("Republisher Node Started. Waiting for /camera/ streams...")

    def color_callback(self, msg):
        try:
            # Convert ROS -> OpenCV
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Apply Transform
            processed_frame = self.crop_and_resize_opencv(cv_image, self.crop_scale, is_depth=False)

            # Publish Raw
            out_msg = self.br.cv2_to_imgmsg(processed_frame, encoding="bgr8")
            out_msg.header = msg.header # Preserve timestamp
            self.pub_color_raw.publish(out_msg)

            # Publish Compressed
            success, encoded_image = cv2.imencode('.jpg', processed_frame)
            if success:
                comp_msg = CompressedImage()
                comp_msg.header = msg.header
                comp_msg.format = 'jpeg'
                comp_msg.data = encoded_image.tobytes()
                self.pub_color_comp.publish(comp_msg)

        except Exception as e:
            self.get_logger().warn(f"Error processing color: {e}")

    def depth_callback(self, msg):
        try:
            # Convert ROS -> OpenCV (16-bit raw depth)
            cv_depth = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Apply Transform
            processed_depth = self.crop_and_resize_opencv(cv_depth, self.crop_scale, is_depth=True)

            # Publish Raw
            out_msg = self.br.cv2_to_imgmsg(processed_depth, encoding="16UC1")
            out_msg.header = msg.header
            self.pub_depth_raw.publish(out_msg)

        except Exception as e:
            self.get_logger().warn(f"Error processing depth: {e}")

    def crop_and_resize_opencv(self, image: np.ndarray, crop_scale: float, is_depth: bool = False) -> np.ndarray:
        assert 0 < crop_scale <= 1.0, "crop_scale must be in (0, 1]"
        
        # Determine dimensions based on input shape (handling both 3-channel and 1-channel/depth)
        if image.ndim == 3:
            H, W, _ = image.shape
        else:
            H, W = image.shape
            
        orig_area = H * W
        target_area = crop_scale * orig_area
        
        # Calculate crop dimensions maintaining aspect ratio of original image
        crop_side = int(np.sqrt(target_area * W / H))
        crop_height = min(crop_side, H)
        crop_width = min(int(crop_side * H / W), W)
        
        # Center crop
        y1 = (H - crop_height) // 2
        x1 = (W - crop_width) // 2
        y2 = y1 + crop_height
        x2 = x1 + crop_width
        
        cropped = image[y1:y2, x1:x2]
        
        # Select interpolation method: Nearest for depth (to preserve values), Area/Linear for color
        method = cv2.INTER_NEAREST if is_depth else cv2.INTER_AREA
        
        resized = cv2.resize(cropped, (self.target_size, self.target_size), interpolation=method)
        return resized
    
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()