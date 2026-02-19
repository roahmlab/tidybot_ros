import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

from sensor_interfaces.msg import SensorState

import tf2_ros
# Importing tf2_geometry_msgs automatically registers the WrenchStamped 
# transform functions required by the tf2 buffer.
import tf2_geometry_msgs 

# Listens to local forces/torques measured by tactile sensors, then
# performs TF lookup to transform into global frame
class TactilePublisher(Node):
    def __init__(self):
        super().__init__('tidybot_tactile_sensor')
        
        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.pub_left = self.create_publisher(WrenchStamped, '/tidybot/contact/left_finger', 10)
        self.pub_right = self.create_publisher(WrenchStamped, '/tidybot/contact/right_finger', 10)
        self.pub_net = self.create_publisher(WrenchStamped, '/tidybot/contact/net_force', 10)
        
        # Subscribers
        self.sub_sensor_0 = self.create_subscription(
            SensorState, '/hub_0/sensor_0', self.sensor_0_cb, 10)
        self.sub_sensor_1 = self.create_subscription(
            SensorState, '/hub_0/sensor_1', self.sensor_1_cb, 10)
            
        # State tracking to accumulate the net force
        self.left_wrench_world = None
        self.right_wrench_world = None
        
    def map_sensor_to_tool(self, x, y, z, is_left):
        """Pre-transforms sensor local frame axes to the tool_frame."""
        if is_left: 
            return float(z), float(-y), float(x)
        else:              
            return float(-z), float(y), float(x)

    def transform_and_publish(self, msg, publisher, is_left):
        # Construct the WrenchStamped message in the tool_frame
        wrench_in = WrenchStamped()
        wrench_in.header.stamp = self.get_clock().now().to_msg()
        wrench_in.header.frame_id = 'tool_frame'
        
        fx, fy, fz = self.map_sensor_to_tool(msg.gfx, msg.gfy, msg.gfz, is_left)
        tx, ty, tz = self.map_sensor_to_tool(msg.gtx * 0.001, msg.gty * 0.001, msg.gtz * 0.001, is_left)

        wrench_in.wrench.force.x = fx
        wrench_in.wrench.force.y = fy
        wrench_in.wrench.force.z = fz
        
        wrench_in.wrench.torque.x = tx
        wrench_in.wrench.torque.y = ty
        wrench_in.wrench.torque.z = tz
        
        try:
            # Look up the raw transform 
            transform = self.tf_buffer.lookup_transform(
                'world', 
                'tool_frame', 
                rclpy.time.Time(), # Get the latest available transform
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            
            # Manually apply the rigid body math to the Wrench
            wrench_out = self.manual_transform_wrench(wrench_in, transform)
            
            # Publish individual finger wrench
            publisher.publish(wrench_out)
            
            # Update internal state
            if is_left:
                self.left_wrench_world = wrench_out
            else:
                self.right_wrench_world = wrench_out
                
            # Attempt to publish the accumulated net force
            self.publish_net_force()
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform from tool_frame to world: {ex}')

    def manual_transform_wrench(self, wrench_in, transform_stamped):
        """Manually applies a TransformStamped to a WrenchStamped."""
        
        # Extract translation and rotation
        tx = transform_stamped.transform.translation.x
        ty = transform_stamped.transform.translation.y
        tz = transform_stamped.transform.translation.z
        
        qx = transform_stamped.transform.rotation.x
        qy = transform_stamped.transform.rotation.y
        qz = transform_stamped.transform.rotation.z
        qw = transform_stamped.transform.rotation.w

        # Helper function for fast quaternion-vector rotation
        def rotate_vector(vx, vy, vz):
            # q_xyz x v
            cx = qy * vz - qz * vy
            cy = qz * vx - qx * vz
            cz = qx * vy - qy * vx
            # (q_xyz x v) + qw * v
            dx = cx + qw * vx
            dy = cy + qw * vy
            dz = cz + qw * vz
            # q_xyz x d
            ex = qy * dz - qz * dy
            ey = qz * dx - qx * dz
            ez = qx * dy - qy * dx
            # Return v + 2 * (q_xyz x d)
            return vx + 2.0 * ex, vy + 2.0 * ey, vz + 2.0 * ez

        # Rotate the force vector
        fx_rot, fy_rot, fz_rot = rotate_vector(
            wrench_in.wrench.force.x, 
            wrench_in.wrench.force.y, 
            wrench_in.wrench.force.z
        )
        
        # Rotate the torque vector
        tx_rot, ty_rot, tz_rot = rotate_vector(
            wrench_in.wrench.torque.x, 
            wrench_in.wrench.torque.y, 
            wrench_in.wrench.torque.z
        )
        
        # Add the torque caused by the translation (Translation Vector cross Rotated Force Vector)
        cross_tx = ty * fz_rot - tz * fy_rot
        cross_ty = tz * fx_rot - tx * fz_rot
        cross_tz = tx * fy_rot - ty * fx_rot
        
        # Build and return the final output message
        w_out = WrenchStamped()
        w_out.header = transform_stamped.header
        w_out.wrench.force.x = fx_rot
        w_out.wrench.force.y = fy_rot
        w_out.wrench.force.z = fz_rot
        w_out.wrench.torque.x = tx_rot + cross_tx
        w_out.wrench.torque.y = ty_rot + cross_ty
        w_out.wrench.torque.z = tz_rot + cross_tz
        
        return w_out

    def sensor_0_cb(self, msg):
        self.transform_and_publish(msg, self.pub_left, is_left=True)
        
    def sensor_1_cb(self, msg):
        self.transform_and_publish(msg, self.pub_right, is_left=False)

    def publish_net_force(self):
        # Ensure we have received at least one message from both sensors
        if self.left_wrench_world is None or self.right_wrench_world is None:
            return
            
        net = WrenchStamped()
        # Adopt the timestamp and frame of the most recently updated left finger
        net.header = self.left_wrench_world.header
        
        # Accumulate forces
        net.wrench.force.x = self.left_wrench_world.wrench.force.x + self.right_wrench_world.wrench.force.x
        net.wrench.force.y = self.left_wrench_world.wrench.force.y + self.right_wrench_world.wrench.force.y
        net.wrench.force.z = self.left_wrench_world.wrench.force.z + self.right_wrench_world.wrench.force.z
        
        # Accumulate torques
        net.wrench.torque.x = self.left_wrench_world.wrench.torque.x + self.right_wrench_world.wrench.torque.x
        net.wrench.torque.y = self.left_wrench_world.wrench.torque.y + self.right_wrench_world.wrench.torque.y
        net.wrench.torque.z = self.left_wrench_world.wrench.torque.z + self.right_wrench_world.wrench.torque.z
        
        self.pub_net.publish(net)

def main(args=None):
    rclpy.init(args=args)
    node = TactilePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()