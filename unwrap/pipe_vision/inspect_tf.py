import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data

class TfInspectorNode(Node):
    def __init__(self):
        super().__init__('tf_inspector')
        self.get_logger().info('TF Inspector Node started. Collecting all frames for 5 seconds...')
        self.frames = set()
        
        # Subscribe to both static and dynamic TF topics
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Create a timer to print results after a delay
        self.timer = self.create_timer(5.0, self.print_results)

    def tf_callback(self, msg):
        """Collect all unique frame IDs from any TF message."""
        for transform in msg.transforms:
            self.frames.add(transform.header.frame_id)
            self.frames.add(transform.child_frame_id)

    def print_results(self):
        """Print the collected unique frames and shut down."""
        self.get_logger().info('-------------------------------------------------')
        self.get_logger().info('Found the following unique frames in /tf and /tf_static:')
        if self.frames:
            for frame in sorted(list(self.frames)):
                self.get_logger().info(f'  - {frame}')
        else:
            self.get_logger().warn('No frames were found.')
        self.get_logger().info('-------------------------------------------------')
        
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TfInspectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()