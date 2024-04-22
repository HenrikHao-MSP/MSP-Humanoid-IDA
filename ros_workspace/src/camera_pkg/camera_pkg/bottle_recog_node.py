import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus
from interfaces.msg import BottleInfo
from constants import RobotStatuses
import re
from nltk import edit_distance

class BottleRecognitionNode(Node):
    def __init__(self):
        super().__init__('bottle_recognition_node')
        self.status_subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10)
        self.bottle_info_publisher = self.create_publisher(
            BottleInfo,
            'bottle_info',
            10)
        self.current_status = 'FINDING_BOTTLE'

    def status_callback(self, msg):
        self.get_logger().info(f'Received status: {msg.status}')
        if msg.status == RobotStatuses.WAITING:  # Replace with the correct status check
            self.current_status = "FINDING_BOTTLE"
            # Activate the recognition processing

            # First find a bottle
            image = self.get_image()
            # Colour Recognition
            self.get_overall_color(image)
            # Word Recognition
            self.get_word(image)

            self.recognize_bottle()

    def get_overall_color(image, k=1):
        """
        Given an image, return the dominant color.
        """
        data = np.reshape(image, (-1, 3))
        data = np.float32(data)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
        _, labels, centers = cv2.kmeans(data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        # Count labels to find the most popular
        counter = Counter(labels.flatten())
        most_popular = counter.most_common(1)[0][0]

        # Convert color back to uint8
        dominant_color = np.uint8(centers[most_popular])

        # Convert from BGR to RGB
        dominant_color = dominant_color[::-1]

        return tuple(dominant_color)

    def get_image():
        # Don't know how to implement the Camera
        return 1

    
    # Regex string to alphabets and lowercase
    def preprocess_string(text):
        # Lowercase the text
        text = text.lower()
        # Remove non-alphabetic characters
        text = re.sub(r'[^a-z]+', '', text)
        return text

    # Using Levenshtein Distance
    def normalised_edit_sim(s1, s2):
        return 1-(edit_distance(s1,s2)/max(len(s1),len(s2)))

    def getHighestProbableWord(self,string):
        # Input String starts here
        accuracy = []
        string = self.preprocess_string(string)
        for target in self.target_string:
            target = self.preprocess_string(target)
            similarity = self.normalised_edit_sim(string, target)
            print(f"Similarity between '{string}' and '{target}': {similarity}")
            accuracy.append((target, similarity))
        max_value = max(accuracy, key = lambda row: row[1])
        return max_value 

    def get_word(image):
        open_cv_image = np.array(image)
    
        open_cv_image = open_cv_image[:, :, ::-1].copy()
        image = open_cv_image
        print(pytesseract.image_to_string(open_cv_image))  
        image = imutils.resize(image, width=2000)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        # Perform text extraction
        detect_text = pytesseract.image_to_string(thresh, lang='eng',config='--psm 6')

        return self.getHighestProbableWord(detect_text)


    

    def recognize_bottle(self):
        # Placeholder for actual recognition logic  
        if self.current_status == "FINDING_BOTTLE":
            
            # Let's say we've recognized a bottle and determined color and position
            bottle_info_msg = BottleInfo()
            bottle_info_msg.color = 'blue'  # Example data
            bottle_info_msg.position = 'A1'  # Example data
            self.bottle_info_publisher.publish(bottle_info_msg)
            self.get_logger().info('Published bottle info')
            
            # You would reset the status if needed or handle subsequent logic
            # self.current_status = "IDLE"

def main(args=None):
    rclpy.init(args=args)

    # Word Recognition Tesseract relative path
    #pytesseract.pytesseract.tesseract_cmd = r"ros_workspace\src\camera_pkg\resource\Tesseract\tesseract.exe"

    # Possible words
    target_string = ["sprite", "vodka", "gin"]

    node = BottleRecognitionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
