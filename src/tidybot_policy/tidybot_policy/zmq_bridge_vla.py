"""
ZMQ Bridge for VLA inference.
Runs on the ROBOT side (bane). Bridges local DDS topics to/from a remote
ZMQ-based VLA inference server over an SSH tunnel.

Usage (inside container on bane):
    python zmq_bridge_vla.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

import zmq
import numpy as np
import threading
import time


class ZMQBridgeVLA(Node):
    def __init__(self):
        super().__init__('zmq_bridge_vla')

        self.latest_image_bytes = None
        self.image_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to camera feed (local DDS)
        self.visual_sub = self.create_subscription(
            CompressedImage,
            '/tidybot/camera_ext/color/compressed',
            self.image_callback,
            qos
        )

        # Publish inferred actions (local DDS)
        self.action_pub = self.create_publisher(
            Float64MultiArray, '/tidybot/arm/delta_commands', 10
        )

        # ZMQ REP socket — waits for requests from inference server
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind('tcp://127.0.0.1:5555')
        self.get_logger().info('ZMQ bridge listening on tcp://127.0.0.1:5555')

        # Start ZMQ server thread
        self._running = True
        self.zmq_thread = threading.Thread(target=self.zmq_loop, daemon=True)
        self.zmq_thread.start()

    def image_callback(self, msg: CompressedImage):
        with self.image_lock:
            self.latest_image_bytes = bytes(msg.data)

    def zmq_loop(self):
        """Serve observations and receive actions over ZMQ."""
        while self._running:
            try:
                req = self.socket.recv_pyobj()

                if req.get('get_obs'):
                    # Send latest camera image
                    with self.image_lock:
                        image_data = self.latest_image_bytes

                    if image_data is not None:
                        self.socket.send_pyobj({
                            'status': 'ok',
                            'image': image_data,
                        })
                    else:
                        self.socket.send_pyobj({
                            'status': 'no_image',
                        })

                elif 'action' in req:
                    # Publish received action to DDS
                    action = req['action']
                    msg = Float64MultiArray()
                    msg.data = action.tolist() if hasattr(action, 'tolist') else list(action)
                    self.action_pub.publish(msg)
                    self.get_logger().info(f'Published action: {msg.data}')
                    self.socket.send_pyobj({'status': 'ok'})

                else:
                    self.socket.send_pyobj({'status': 'unknown_request'})

            except Exception as e:
                self.get_logger().error(f'ZMQ error: {e}')
                time.sleep(0.1)


def main():
    rclpy.init()
    node = ZMQBridgeVLA()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
