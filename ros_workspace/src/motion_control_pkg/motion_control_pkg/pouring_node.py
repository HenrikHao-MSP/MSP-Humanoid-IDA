import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# 
POURING_POSITION = 1500

class PouringControlNode(Node):
    def __init__(self):
        super().__init__('pouring_control_node')
        self.subscription = self.create_subscription(
            Float32,
            'liquid_level',
            self.liquid_level_callback,
            10)
        self.liquid_level_threshold = 80.0  # Example threshold, adjust as needed

    def liquid_level_callback(self, msg):
        current_level = msg.data
        self.get_logger().info(f'Current liquid level: {current_level}%')
        if current_level > self.liquid_level_threshold:
            self.stop_pouring()

    def stop_pouring(self):
        # Placeholder for actual arm stop command
        self.get_logger().info('Liquid level threshold exceeded. Stopping pouring...')
        # Insert your arm control logic here to stop pouring

def main(args=None):
    rclpy.init(args=args)
    node = PouringControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
