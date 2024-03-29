import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus
from constants import RobotStatuses


class MicNode(Node):
    def __init__(self):
        super().__init__('mic_node')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)  # Timer to call publish_status every 1 second
        


    def publish_status(self):
        # Create an instance of the custom message
        msg = RobotStatus()
        
        msg.status = RobotStatuses.POURING
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published status
        self.get_logger().info(f'Publishing Robot Status: "{msg.status}"')

def main(args=None):
    rclpy.init(args=args)
    node = MicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

