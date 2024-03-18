import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from skimage import io
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.measure import label, regionprops
from skimage.morphology import binary_opening, disk
import numpy as np
import matplotlib.pyplot as plt

class PouringAccuracyNode(Node):
    def __init__(self):
        super().__init__('pouring_accuracy_node')
        self.subscription = self.create_subscription(
            Bool,
            'arm_move_success',  # Listen to the success flag from ArmControlNode
            self.success_callback,
            10)
        self.liquid_level_publisher = self.create_publisher(
            Float32,
            'liquid_level',  # Topic for publishing the liquid level
            10)
        
    def success_callback(self, msg):
        if msg.data:  # True if arm movement was successful
            self.get_logger().info('Arm movement was successful, analyzing pouring accuracy...')
            self.analyze_pouring_accuracy()
        else:
            self.get_logger().info('Arm movement was not successful, skipping pouring accuracy analysis.')

    def analyze_pouring_accuracy(self):
        # Your image processing logic here
        empty_cup_path = '/path/to/your/empty_cup_image.jpg'  # Update this path
        filled_cup_path = '/path/to/your/filled_cup_image.jpg'  # Update this path
        
        empty_cup_image = io.imread(empty_cup_path)
        filled_cup_image = io.imread(filled_cup_path)
        
        empty_cup_gray = rgb2gray(empty_cup_image)
        filled_cup_gray = rgb2gray(filled_cup_image)

        liquid_level, labeled_mask = self.find_liquid_level(filled_cup_gray, empty_cup_gray)
        
        liquid_height_percentage_final = ((filled_cup_gray.shape[0] - liquid_level) / filled_cup_gray.shape[0]) * 100
        

        # Create a message and publish the liquid level percentage
        liquid_level_msg = Float32()
        liquid_level_msg.data = liquid_height_percentage_final
        self.liquid_level_publisher.publish(liquid_level_msg)
        
        self.get_logger().info(f"Published liquid level: {liquid_height_percentage_final}% full.")


    def find_liquid_level(self, filled_cup_gray, empty_cup_gray):
        difference_image = np.abs(filled_cup_gray - empty_cup_gray)
        thresh = threshold_otsu(difference_image)
        binary_difference = difference_image > thresh
        selem = disk(3)
        clean_binary_difference = binary_opening(binary_difference, selem)
        labeled_mask = label(clean_binary_difference)
        regions = regionprops(labeled_mask)
        
        if regions:
            largest_region = max(regions, key=lambda r: r.area)
            liquid_level = largest_region.bbox[0]
        else:
            liquid_level = filled_cup_gray.shape[0]
        
        return liquid_level, labeled_mask

def main(args=None):
    rclpy.init(args=args)
    node = PouringAccuracyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
