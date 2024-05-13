import rclpy
from rclpy.node import Node
import cv2
import time
from sensor_msgs.msg import Image as Im
from std_msgs.msg import String
from cv_bridge import CvBridge
import tensorflow as tf
from tensorflow.keras.models import Sequential
import numpy as np

# Load the trained TensorFlow/Keras model and class names
model = tf.keras.models.load_model("/home/zuhayr/Desktop/keras_model.h5")
class_names = open("/home/zuhayr/Desktop/labels.txt", "r").readlines()

# Initialize CvBridge for converting between OpenCV images and ROS Image messages
bridge = CvBridge()

class DetectImage(Node):

    def __init__(self):
        super().__init__('rec_camera')
        # Create a subscription to the 'camera' topic with the callback function 'camera_image_ai'
        self.subscription = self.create_subscription(Im, 'camera', self.camera_image_ai, 10)
        # Create a publisher for the 'arduino_command' topic
        self.publisher = self.create_publisher(String, 'arduino_command', 10)

    def camera_image_ai(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg)

        # Prepare the image for the model
        data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
        image = cv2.resize(cv_image, (224, 224))
        image_array = np.asarray(image)
        normalized_image_array = (image_array.astype(np.float32) / 127.5) - 1
        data[0] = normalized_image_array

        # Make a prediction using the loaded model
        prediction = model.predict(data)
        index = np.argmax(prediction)
        class_name = class_names[index]
        confidence_score = prediction[0][index]

         # Extract individual class probabilities
        class_1 = prediction[0, 0]
        class_2 = prediction[0, 1]
        class_3  = prediction[0, 2]
        class_4  = prediction[0, 3]
        class_5  = prediction[0, 4]
        class_6  = prediction[0, 5]

        # Find the maximum predicted class and check confidence score
        predMax = max(class_1, class_2, class_3, class_4, class_5, class_6)

        # Publish corresponding command based on the prediction
        if predMax == class_1 and confidence_score > 0.8:
            msg = String()
            msg.data = "zuhayr"
            self.publisher.publish(msg)
            self.get_logger().info('zuhayr')

        elif predMax == class_2 and confidence_score > 0.8:
            msg = String()
            msg.data = "nobody"
            self.publisher.publish(msg)
            self.get_logger().info('nobody')

        elif predMax == class_3 and confidence_score > 0.8:
            msg = String()
            msg.data = "glasses"
            self.publisher.publish(msg)
            self.get_logger().info('glasses')

        elif predMax == class_4 and confidence_score > 0.8:
            msg = String()
            msg.data = "woman"
            self.publisher.publish(msg)
            self.get_logger().info('woman')

        elif predMax == class_5 and confidence_score > 0.8:
            msg = String()
            msg.data = "tom"
            self.publisher.publish(msg)
            self.get_logger().info('tom')

        elif predMax == class_6 and confidence_score > 0.8:
            msg = String()
            msg.data = "robert"
            self.publisher.publish(msg)
            self.get_logger().info('robert')

        else:
            msg = String()
            msg.data = "intruder"
            self.publisher.publish(msg)
            self.get_logger().info('intruder')

    def send_arduino_command(self, command):
        command
        

def main(args=None):
    # Initialize the ROS Python client library
    rclpy.init(args=args)
    # Create an instance of the DetectImage class
    detect_image = DetectImage()
    # Enter the ROS event loop
    rclpy.spin(detect_image)
    # Destroy the ROS node
    detect_image.destroy_node()
    # Shut down the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Call the main function if the script is executed directly
    main()