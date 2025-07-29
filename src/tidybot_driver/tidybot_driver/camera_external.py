import rclpy
from rclpy.node import Node
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Camera(Node):
    def __init__(self):
        super().__init__('tidybot_camera')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            CompressedImage, 
            '/tidybot/camera/color/compressed', 
            qos
        )
        self.br = CvBridge()

        # Use webcam
        self.cap = cv2.VideoCapture(1)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam (/dev/video0)")
            return

        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Webcam resolution: {w}x{h}")

        # Set your desired FPS
        fps = 10.0
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from webcam")
            return

        success, encoded_image = cv2.imencode('.jpg', frame)
        if not success:
            self.get_logger().warn("JPEG compression failed")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        self.publisher_.publish(msg)
        self.get_logger().debug("Published webcam image")

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
