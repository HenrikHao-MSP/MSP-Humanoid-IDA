import rclpy 
from rclpy.node import Node
from depthai_ros_msgs.msg import SpatialDetectionArray  # Make sure you have the correct import for the message type
from interfaces.msg import DetectionInfo
from interfaces.msg import DetectionInfoArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import difflib
# Word
import pytesseract
import imutils

class SpatialDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('spatial_detection_subscriber')
        self.latest_image = None
        self.bridge = CvBridge()

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

        self.info_publisher = self.create_publisher(DetectionInfoArray, 'detection_info', 10)
        self.beverage_list = ['Milk', 'Vodka']
    
    def image_listener_callback(self, msg):
        # Convert ROS Image message to CV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image

    def process_detection(self, detection, object_name):
        bbox = detection.bbox
        center_x, center_y = int(bbox.center.x), int(bbox.center.y)
        size_x, size_y = int(bbox.size_x), int(bbox.size_y)
        x_start, x_end = max(0, center_x - size_x // 2), min(self.latest_image.shape[1], center_x + size_x // 2)
        y_start, y_end = max(0, center_y - size_y // 2), min(self.latest_image.shape[0], center_y + size_y // 2)
        
        # Extract the region of interest
        roi = self.latest_image[y_start:y_end, x_start:x_end]

        if object_name == 'bottle':
            dominant_color = self.find_dominant_color(roi)
            word = self.get_word(roi)
        else:  # For other objects, you might have different processing logic
            dominant_color = [0, 0, 0]
            word = None
            if object_name == 'cup':
                image_filename = '/home/henrik/MSP-Humanoid-IDA/ros_workspace/src/camera_pkg/resource/baseimg.jpg'
                cv2.imwrite(image_filename, roi)

        # Create DetectionInfo object
        detection_info = DetectionInfo()
        detection_info.name = object_name
        detection_info.x = float(detection.position.x)
        detection_info.y = float(detection.position.y)
        detection_info.z = float(detection.position.z)
        detection_info.text = str(word)
        detection_info.color = dominant_color
        if object_name == 'bottle':
            self.get_logger().info(f'Bottle Position: x: {detection_info.x}, y: {detection_info.y}, z: {detection_info.z}, color: {detection_info.color}, name: {detection_info.text}')
        return detection_info

    def detection_listener_callback(self, msg):
        if self.latest_image is not None:
            all_detections = []

            for detection in msg.detections:
                for result in detection.results:
                    if result.id == '39':  # Bottle
                        detection_info = self.process_detection(detection, 'bottle')
                        all_detections.append(detection_info)
                    elif result.id == '41':  # Cup
                        detection_info = self.process_detection(detection, 'cup')
                        all_detections.append(detection_info)

            # Publish all detections as a list
            if all_detections:
                detection_info_array = DetectionInfoArray()
                detection_info_array.detections = all_detections
                self.info_publisher.publish(detection_info_array)
                self.get_logger().info(f'Published {len(all_detections)} detections')

    def find_dominant_color(self, image):
        # Convert image to RGB (OpenCV uses BGR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Reshape the image to be a list of pixels
        pixels = image.reshape((-1, 3))
        # Convert from integers to floats
        pixels = np.float32(pixels)
        # Define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
        _, labels, centers = cv2.kmeans(pixels, 1, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        # Convert back to 8 bit values
        centers = np.uint8(centers)
        # Get the dominant color
        dominant_color = centers[0].tolist()
        return dominant_color

    def get_word(self, image):
        # Resize the image to make it more suitable for text detection
        image = imutils.resize(image, width=2000)
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Use adaptive thresholding
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV, 11, 2)
        # Configuration for Tesseract
        custom_config = r'--oem 3 --psm 6'
        word = pytesseract.image_to_string(thresh, lang='eng', config=custom_config)
        # Find the closest match from the beverage list
        matches = difflib.get_close_matches(word.strip(), self.beverage_list, n=1, cutoff=0)
        # Perform text extraction
        return matches[0] if matches else None


def main(args=None):
    rclpy.init(args=args)
    spatial_detection_subscriber = SpatialDetectionSubscriber()
    rclpy.spin(spatial_detection_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    spatial_detection_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()