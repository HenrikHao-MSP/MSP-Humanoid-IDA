import rclpy
from rclpy.node import Node
from interfaces.msg import BottleInfo
from std_msgs.msg import Bool

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.subscription = self.create_subscription(
            BottleInfo,
            'bottle_info',
            self.bottle_info_callback,
            10)
        self.publisher_ = self.create_publisher(
            Bool,
            'arm_move_success',  # Topic name for the success flag
            10)
        self.subscription  # prevent unused variable warning

    def bottle_info_callback(self, msg):
        self.get_logger().info(f'Received bottle info: {msg.position}')
        # Control logic to move the arm to the bottle's position
        success = self.move_arm_to_position(msg.position)
        # Publish the success flag
        success_msg = Bool()
        success_msg.data = success
        self.publisher_.publish(success_msg)
        self.get_logger().info(f'Published move success flag: {success}')

    def move_arm_to_position(self, position):
        # Placeholder for arm control logic
        # Here, simulate moving the arm and return True for success
        self.get_logger().info(f'Moving arm to position: {position}')
        # Simulate a successful operation
        return True

def main(args=None):
    rclpy.init(args=args)
    arm_control_node = ArmControlNode()
    rclpy.spin(arm_control_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
