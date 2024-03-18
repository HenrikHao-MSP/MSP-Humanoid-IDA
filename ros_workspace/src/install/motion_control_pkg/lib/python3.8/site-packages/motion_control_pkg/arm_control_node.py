import rclpy
from rclpy.node import Node
from interfaces.msg import BottleInfo 

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.subscription = self.create_subscription(
            BottleInfo,
            'bottle_info',
            self.bottle_info_callback,
            10)
        self.subscription  # prevent unused variable warning

    def bottle_info_callback(self, msg):
        self.get_logger().info(f'Received bottle info: {msg.position}')
        # Control logic to move the arm to the bottle's position
        self.move_arm_to_position(msg.position)

    def move_arm_to_position(self, position):
        # Placeholder for arm control logic
        # Convert 'position' to specific coordinates or control commands for your robotic arm
        # Implement the commands to control the arm based on the provided position
        pass

def main(args=None):
    rclpy.init(args=args)
    arm_control_node = ArmControlNode()
    rclpy.spin(arm_control_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
