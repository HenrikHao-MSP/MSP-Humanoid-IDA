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
from depthai_ros_msgs.msg import SpatialDetectionArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PouringAccuracyNode(Node):
    def __init__(self):
        super().__init__('pouring_accuracy_node')
        self.latest_image = None
        self.bridge = CvBridge()
        self.empty_cup = '/home/henrik/MSP-Humanoid-IDA/ros_workspace/src/camera_pkg/resource/baseimg.jpg'
        self.arm_mvoe_success = False
        self.subscription = self.create_subscription(
            Bool,
            'arm_move_success',  # Listen to the success flag from ArmControlNode
            self.success_callback,
            10)
        
        self.detection_subscription = self.create_subscription(
            SpatialDetectionArray,
            'color/yolov4_Spatial_detections',
            self.detection_listener_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            'color/image',
            self.image_listener_callback,
            10)
        
        self.liquid_level_publisher = self.create_publisher(
            Float32,
            'liquid_level',  # Topic for publishing the liquid level
            10)
        
    def success_callback(self, msg):
        self.arm_move_success = msg.data
        if self.arm_move_success:  # True if arm movement was successful
            self.get_logger().info('Arm movement was successful, analyzing pouring accuracy...')
        else:
            self.get_logger().info('Arm movement was not successful, skipping pouring accuracy analysis.')

    def image_listener_callback(self, msg):
        # Convert ROS Image message to CV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image
    
    def detection_listener_callback(self, msg):
        if self.arm_move_success:
            if self.latest_image is not None:
                for detection in msg.detections:
                    for result in detection.results:
                        if result.id == '41':  # Bottle
                            bbox = detection.bbox
                            center_x, center_y = int(bbox.center.x), int(bbox.center.y)
                            size_x, size_y = int(bbox.size_x), int(bbox.size_y)
                            x_start, x_end = max(0, center_x - size_x // 2), min(self.latest_image.shape[1], center_x + size_x // 2)
                            y_start, y_end = max(0, center_y - size_y // 2), min(self.latest_image.shape[0], center_y + size_y // 2)
                            roi = self.latest_image[y_start:y_end, x_start:x_end]
                            self.analyze_pouring_accuracy(roi, self.empty_cup)

        # Extract the region of interest
        roi = self.latest_image[y_start:y_end, x_start:x_end]
    def analyze_pouring_accuracy(self, img, empty_cup):

        empty_cup_image = io.imread(empty_cup)
        filled_cup_image = img
        
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
