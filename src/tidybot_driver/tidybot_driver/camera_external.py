import rclpy
from rclpy.node import Node
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class Camera(Node):
    def __init__(self):
        super().__init__('tidybot_ext_camera')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.raw_publisher_ = self.create_publisher(
            Image, 
            '/tidybot/camera_ext/color/raw', 
            qos
        )

        self.publisher_ = self.create_publisher(
            CompressedImage, 
            '/tidybot/camera_ext/color/compressed', 
            qos
        )
        self.br = CvBridge()

        # Use webcam
        self.cap = cv2.VideoCapture(4)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam (/dev/video4)")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Webcam resolution: {w}x{h}")

        fps = 5
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from webcam")
            return
        frame = self.crop_and_resize_opencv(frame, crop_scale=1, output_size=(224, 224))

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

        self.raw_publisher_.publish(msg_raw)
        self.publisher_.publish(msg)
        self.get_logger().info("Published webcam image")
    
    @staticmethod
    def crop_and_resize_opencv(image: np.ndarray, crop_scale: float, output_size=(224, 224)) -> np.ndarray:
        """
        Center-crops a single image to have area `crop_scale` * (original image area),
        then resizes to `output_size`.

        Args:
            image: np.ndarray of shape (H, W, C) with dtype=np.uint8 or np.float32, range [0,255] or [0,1]
            crop_scale: float in (0,1] indicating what fraction of the original image area to crop.
            output_size: tuple (H, W) to resize cropped image to.

        Returns:
            Cropped and resized image as np.ndarray of shape (output_size[0], output_size[1], C)
        """
        assert 0 < crop_scale <= 1.0, "crop_scale must be in (0, 1]"
        assert image.ndim == 3 and image.shape[2] == 3, "Input must be (H, W, 3)"

        H, W, _ = image.shape
        orig_area = H * W
        target_area = crop_scale * orig_area
        crop_side = int(np.sqrt(target_area * W / H))  # preserve aspect ratio

        crop_height = min(crop_side, H)
        crop_width = min(int(crop_side * H / W), W)

        y1 = (H - crop_height) // 2
        x1 = (W - crop_width) // 2
        y2 = y1 + crop_height
        x2 = x1 + crop_width

        cropped = image[y1:y2, x1:x2]
        resized = cv2.resize(cropped, output_size, interpolation=cv2.INTER_AREA)

        return resized

    def destroy_node(self):
        if self.cap:
            self.cap.release()
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
