import rclpy 
from rclpy.node import Node
from depthai_ros_msgs.msg import SpatialDetectionArray  # Make sure you have the correct import for the message type
from interfaces.msg import DetectionInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

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

        self.info_publisher = self.create_publisher(DetectionInfo, 'detection_info', 10)
    def image_listener_callback(self, msg):
        # Convert ROS Image message to CV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image

    def detection_listener_callback(self, msg):
        if self.latest_image is not None:
            for detection in msg.detections:
                for result in detection.results:
                    if result.id == '39':  # Check if the id is 'bottle'
                        # Process detection's bounding box to find dominant color
                        bbox = detection.bbox
                        center_x, center_y = int(bbox.center.x), int(bbox.center.y)
                        size_x, size_y = int(bbox.size_x), int(bbox.size_y)
                        x_start, x_end = max(0, center_x - size_x // 2), min(self.latest_image.shape[1], center_x + size_x // 2)
                        y_start, y_end = max(0, center_y - size_y // 2), min(self.latest_image.shape[0], center_y + size_y // 2)
                        
                        # Extract the region of interest
                        roi = self.latest_image[y_start:y_end, x_start:x_end]
                        
                        # Determine the dominant color
                        dominant_color = self.find_dominant_color(roi)
                        word = self.get_word(roi)
                        info_msg = DetectionInfo()
                        info_msg.x = center_x
                        info_msg.y = center_y
                        info_msg.text = word
                        info_msg.color = dominant_color
                        cv_image_with_bbox = cv2.rectangle(self.latest_image.copy(), (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
                        image_filename = f'/home/henrik/MSP-Humanoid-IDA/ros_workspace/src/camera_pkg/resource/detection_{center_x}_{center_y}.jpg'
                        cv2.imwrite(image_filename, cv_image_with_bbox)
                        # Publish the message
                        self.info_publisher.publish(info_msg)
                        self.get_logger().info(f'Published detection info: {info_msg}')
                    #if result.id == '41':
                    #    bbox = detection.bbox

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
        # Apply thresholding to binarize the image
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

        # Perform text extraction
        return pytesseract.image_to_string(thresh, lang='eng', config='--psm 6')


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
