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

class Camera(Node):
    def __init__(self):
        super().__init__('tidybot_wrist_camera')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_width', 224)
        self.declare_parameter('image_height', 224)
        self.declare_parameter('read_timeout_sec', 0.1)  # seconds to wait for read()

        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.read_timeout_sec = self.get_parameter('read_timeout_sec').get_parameter_value().double_value

        self.publisher_raw_ = self.create_publisher(Image, '/tidybot/camera_wrist/color/raw', qos)
        self.publisher_ = self.create_publisher(CompressedImage, '/tidybot/camera_wrist/color/compressed', qos)
        self.br = CvBridge()

        # Thread pool for timed reads (not to be confused with the rclpy executor)
        self.read_thread_pool = ThreadPoolExecutor(max_workers=1)

        self.pipeline = (
            'rtspsrc location=rtsp://192.168.1.10/color protocols=tcp latency=0 '
            'tcp-timeout=5000000 timeout=5000000 '
            '! rtph264depay '
            '! queue max-size-buffers=1 leaky=downstream '
            '! avdec_h264 '
            '! videoconvert '
            '! appsink drop=true sync=false'
        )
        self.get_logger().info("Opening RTSP stream from wrist camera with pipeline:\n" + self.pipeline)
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error(f"{RED}{BOLD}Failed to open RTSP stream{RESET}")
            return
        self.get_logger().info(f"{GREEN}{BOLD}Successfully opened RTSP stream{RESET}")

        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Wrist camera resolution: {w}x{h}")

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timed_read(self):
        """Helper to wrap cap.read() so it can be cancelled if blocking."""
        return self.cap.read()

    def reconnect(self):
        self.get_logger().warn(f"{YELLOW}Re-connecting RTSP stream...{RESET}")
        try:
            self.cap.release()
        except Exception:
            pass
        time.sleep(1.0)
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            self.get_logger().info(f"{GREEN}{BOLD}Reconnected to RTSP stream{RESET}")
        else:
            self.get_logger().error(f"{RED}{BOLD}Reconnect failed{RESET}")

    def timer_callback(self):
        # Submit read to thread pool and wait with timeout
        # Use a ThreadPoolExecutor as a workaround for OpenCV's blocking cap.read()
        future = self.read_thread_pool.submit(self.timed_read)
        try:
            ret, frame = future.result(timeout=self.read_timeout_sec)
        except TimeoutError:
            self.get_logger().warn(f"{YELLOW}cap.read() timed out after {self.read_timeout_sec:.3f}s{RESET}")
            self.reconnect()
            return
        except Exception as e:
            self.get_logger().warn(f"{YELLOW}Exception during cap.read(): {e}{RESET}")
            self.reconnect()
            return

        if not ret or frame is None:
            self.get_logger().warn(f"{YELLOW}Failed to read frame from RTSP stream (ret={ret}){RESET}")
            self.reconnect()
            return

        # Process frame
        frame = self.crop_and_resize_opencv(frame, crop_scale=1.0)

        success, encoded_image = cv2.imencode('.jpg', frame)
        if not success:
            self.get_logger().warn("JPEG compression failed")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        msg_raw = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        msg_raw.header.stamp = self.get_clock().now().to_msg()

        self.publisher_raw_.publish(msg_raw)
        self.publisher_.publish(msg)

    def crop_and_resize_opencv(self, image: np.ndarray, crop_scale: float) -> np.ndarray:
        assert 0 < crop_scale <= 1.0, "crop_scale must be in (0, 1]"
        assert image.ndim == 3 and image.shape[2] == 3, "Input must be (H, W, 3)"
        H, W, _ = image.shape
        orig_area = H * W
        target_area = crop_scale * orig_area
        crop_side = int(np.sqrt(target_area * W / H))
        crop_height = min(crop_side, H)
        crop_width = min(int(crop_side * H / W), W)
        y1 = (H - crop_height) // 2
        x1 = (W - crop_width) // 2
        y2 = y1 + crop_height
        x2 = x1 + crop_width
        cropped = image[y1:y2, x1:x2]
        resized = cv2.resize(cropped, (self.image_width, self.image_height), interpolation=cv2.INTER_AREA)
        return resized

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        self.read_thread_pool.shutdown(wait=False)
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
