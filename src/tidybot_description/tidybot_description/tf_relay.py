import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage

qos_static = QoSProfile(depth=10)
qos_static.reliability = ReliabilityPolicy.RELIABLE
qos_static.durability  = DurabilityPolicy.TRANSIENT_LOCAL

class TfRelay(Node):
    def __init__(self):
        """
        A relay node that take over the /tf and /tf_static topics, and republishes them
        on /tf_relay and /tf_static_relay respectively. Used for reset the TF buffer
        in RViz when the world is reset, and records the transforms in a episode.
        """
        super().__init__('tf_relay')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 1) Dynamic TF: reliable, volatile
        qos_dyn = QoSProfile(depth=10)
        qos_dyn.reliability = ReliabilityPolicy.RELIABLE
        qos_dyn.durability  = DurabilityPolicy.VOLATILE
        self.pub_tf     = self.create_publisher(TFMessage, '/tf_relay',       qos_dyn)

       # 2) Static TF: reliable, transient-local
        qos_stat = QoSProfile(depth=10)
        qos_stat.reliability = ReliabilityPolicy.RELIABLE
        qos_stat.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_static = self.create_publisher(TFMessage, '/tf_static_relay', qos_stat)

        self.sub_tf     = self.create_subscription(TFMessage, '/tf',        self.handle_tf,    qos_profile_sensor_data)
        self.sub_static = self.create_subscription(TFMessage, '/tf_static', self.handle_static, qos_static)

        self.create_service(Empty, 'reset_tf_buffer', self.reset_buffer_cb)

    def handle_tf(self, msg: TFMessage):
        # Simply republish the incoming transforms
        self.get_logger().debug(f'Received {len(msg.transforms)} dynamic transforms')
        self.pub_tf.publish(msg)

    def handle_static(self, msg: TFMessage):
        # Republish static transforms once, so RViz’s Static TF display can pick them up
        self.pub_static.publish(msg)

    def reset_buffer_cb(self, request, response):
        # Clear both dynamic and static entries
        self.tf_buffer.clear()
        self.get_logger().info('TF buffer cleared')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
