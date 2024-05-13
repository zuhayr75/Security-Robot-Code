import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):

    def __init__(self):
        super().__init__('get_camera')
        # Create a publisher to publish images with a queue size of 10
        self.publisher = self.create_publisher(Image, 'camera', 10)
        
        # Set the timer period for capturing frames (66 milliseconds)
        timer_period = 0.066
        # Create a timer that calls the get_cam_frame method periodically
        self.timer = self.create_timer(timer_period, self.get_cam_frame)
        
        # Initialize frame counter
        self.i = 0
        # Open a video capture device (camera index 0)
        self.vid = cv2.VideoCapture(0)
        # Initialize CvBridge for converting between OpenCV images and ROS Image messages
        self.bridge = CvBridge()

        # Load the Haar cascade for face detection
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    def __del__(self):
        # Release the video capture device in the destructor
        self.vid.release()

    def get_cam_frame(self):
        # Check if the video capture device is opened
        if not self.vid.isOpened():
            self.get_logger().warn('Video capture not opened!')
            return

        # Read a frame from the video capture device
        ret, frame = self.vid.read()
        # Check if the frame reading was successful
        if not ret:
            self.get_logger().warn('Failed to read frame from the camera!')
            return

        # Convert the frame to grayscale for face detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces in the frame using the Haar cascade
        faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=1.3, minNeighbors=5)

        # Draw rectangles around the detected faces
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Display the frame in a window named 'frame'
        cv2.imshow('frame', frame)
        
        # Check for the 'q' key to quit the application
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.vid.release()  # Release the video capture device
            cv2.destroyAllWindows()  # Close the OpenCV window
            rclpy.shutdown()  # Shut down the ROS node

        # Convert the OpenCV frame to a ROS Image message
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        try:
            # Publish the ROS Image message
            self.publisher.publish(image_message)
            self.get_logger().info('Publishing image')
        except Exception as e:
            print(e)

def main(args=None):
    # Initialize the ROS Python client library
    rclpy.init(args=args)
    # Create an instance of the CameraNode class
    node = CameraNode()
    # Enter the ROS event loop
    rclpy.spin(node)
    # Shut down the ROS client library
    rclpy.shutdown()

# Entry point for the script
if __name__ == '__main__':
    # Call the main function if the script is executed directly
    main()