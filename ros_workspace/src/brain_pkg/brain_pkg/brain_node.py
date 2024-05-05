import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus
from interfaces.msg import BottleInfo
from constants import RobotStatuses
import re
from nltk import edit_distance

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.status = RobotStatuses.WAITING
        # placeholder for chatbot subscription
        self.bottle_recog_subscription = self.create_subscription(BottleInfo, 'bottle_info', self.bottle_info_callback, 10)
        self.arm_subscription = self.create_subscription(MSG_TYPE, TOPIC_NAME, self.arm_info_callback, 10)

    def bottle_info_callback(self, msg:BottleInfo):
        if msg.color and msg.position:
            self.status = RobotStatuses.PICK_UP

    def arm_info_callback(self, msg):
        # when receiving the msg, converting self.status to pour or putdown
        return

def main(args=None):
    rclpy.init(args=args)

    node = BrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
