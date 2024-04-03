import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus
from constants import RobotStatuses
import openai
import speech_recognition as sr
import random
import pyttsx3

# Set your OpenAI API key
openai.api_key = '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'

# Initialize the recognizer
r = sr.Recognizer()

# Initialize the speech recognizer and text-to-speech engine
recognizer = sr.Recognizer()
text_to_speech = pyttsx3.init()

# Define the initial messages
messages = [
    {"role": "system", "content": "You are Ida, a witty bartender with a sharp sense of humor."},
    {"role": "user", "content": "Hey Bartender, what's your recommendation tonight?"},
    {"role": "assistant", "content": "Well, well, well! Tonight's star is the 'Sarcastic Sunrise.' It's like a regular sunrise, but with a twist of sarcasm."},
]

# Define drink flavors
flavors_dict = {
    "red drink": ["strawberry", "watermelon"],
    "red beverage": ["strawberry", "watermelon"],
    "red": ["strawberry", "watermelon"],
    "blue drink": ["blueberry"],
    "blue beverage": ["blueberry"],
    "blue": ["blueberry"],
    "green drink": ["green apple", "mint"],
    "green beverage": ["green apple", "mint"],
    "green": ["green apple", "mint"],
}

def order_beverage(user_input):
    for color, flavors in flavors_dict.items():
        if color in user_input:
            chosen_flavor = random.choice(flavors)
            response = f"Sure! I'll give you a bottle of {color} with {chosen_flavor} flavor."
            return response
    response = "I can only prepare 3 kinds of color drinks for you. Please tell me what color drink you want."
    return response

def is_request(sentence):
    phrases = ['can I', 'could you', 'can you', 'I would like', 'I want']
    for phrase in phrases:
        if phrase.lower() in sentence.lower():
            return True
    return False

def is_not_colour_drink(sentence):
    phrases = ['wine', 'coffee', 'cocktail']
    for phrase in phrases:
        if phrase.lower() in sentence.lower():
            return True
    return False

def recognize_speech_from_mic(recognizer, microphone):
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    with microphone as source:
        print("Customer speaking...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        return recognizer.recognize_google(audio)
    except sr.RequestError:
        print("API unavailable")
    except sr.UnknownValueError:
        print("Unable to recognize speech")

def openai_generate_response(prompt):
    response = openai.Completion.create(
        engine="gpt-3.5-turbo-instruct",  # Use the recommended replacement model
        prompt=prompt,
        temperature=0.5,
        max_tokens=100,  
    )
    return response.choices[0].text.strip()

# Function for text-to-speech
def text_to_speech_function(text):
    text_to_speech.setProperty('rate', 150)  # Adjust the rate to speak faster or slower
    text_to_speech.say(text)
    text_to_speech.runAndWait()

class MicNode(Node):
    def __init__(self):
        super().__init__('mic_node')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
    def publish_status(self):
        speech_text = recognize_speech_from_mic(recognizer, self.mic)
        if speech_text:
            print(f"You said: {speech_text}")

            if any(phrase in speech_text.lower() for phrase in ["goodbye", "see you", "bye"]):
                print("Goodbye!")
                text_to_speech_function("Goodbye!")
                # Here you might want to send a command to stop the robot if necessary
                # e.g., self.send_stop_command()
                return

            if "order" in speech_text.lower() and not is_not_colour_drink(speech_text.lower()):
                if any(color in speech_text.lower() for color in ["red", "blue", "green"]):
                    response = order_beverage(speech_text.lower())
                    print(response)
                    text_to_speech_function(response)
                else:
                    print("I can only prepare red, blue, or green drinks for you. Please tell me what color drink you want.")
                    text_to_speech_function("I can only prepare red, blue, or green drinks for you. Please tell me what color drink you want.")
            else:
                prompt = f"{messages[-2]['content']} You said: {speech_text} Assistant:"
                response_text = openai_generate_response(prompt)
                print(f"Assistant: {response_text}")
                text_to_speech_function(response_text)

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

