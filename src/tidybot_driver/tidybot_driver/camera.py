import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Camera(Node):
    def __init__(self):
        super().__init__('tidybot_camera')

        self.publisher_ = self.create_publisher(Image, '/tidybot/camera/color', 10)
        self.br = CvBridge()

        # RTSP stream URL
        self.rtsp_url = 'rtsp://192.168.1.10/color'

        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open RTSP stream at {self.rtsp_url}")
            return

        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from RTSP stream')
            return

        # Convert and publish the frame
        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

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
