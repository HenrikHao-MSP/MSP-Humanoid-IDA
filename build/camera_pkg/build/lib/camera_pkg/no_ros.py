import imutils
import cv2
import pytesseract
import numpy as np

def get_word(image):
    open_cv_image = np.array(image)

    open_cv_image = open_cv_image[:, :, ::-1].copy()
    image = open_cv_image
    print(pytesseract.image_to_string(open_cv_image))  
    image = imutils.resize(image, width=2000)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    # Perform text extraction
    return pytesseract.image_to_string(thresh, lang='eng',config='--psm 6')

def image_to_cv2(image):
    """Convert image to cv2"""
    file_bytes = image.read()

    # Convert the byte stream into a NumPy array compatible with OpenCV
    image = cv2.imdecode(np.frombuffer(file_bytes, np.uint8), cv2.IMREAD_COLOR)
    return image

# Word Recognition Tesseract relative path
pytesseract.pytesseract.tesseract_cmd = r"ros_workspace\src\camera_pkg\resource\Tesseract\tesseract.exe"

# Import image, not necessary if there is roi
image = open('ros_workspace/src/camera_pkg/resource/Fonts.jpeg', 'rb')

# If image is not in cv2 format
image = image_to_cv2(image)

# Word Recognition section
print(get_word(image))

# If roi is already in cv2 format, just get_word(roi)
# If not then run image_to_cv2, to get cv2

