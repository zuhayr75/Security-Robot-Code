import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class arduinoControl(Node):

    def __init__(self):
        super().__init__('arduino_control')
        self.subscription = self.create_subscription(String, 'arduino_command', self.send_command_to_arduino, 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 9600)  # Change the port accordingly

    def send_command_to_arduino(self, msg):
        print("Received ", msg.data)
        command = msg.data

        if command == "zuhayr":
            self.serial_port.write(b'1')  # Sending '2' to turn on right 
            self.get_logger().info("Sent 'zuhayr' command to Arduino")
        elif command == "nobody":
            self.serial_port.write(b'2')  # Sending '1' to turn on left
            self.get_logger().info("Sent 'nobody' command to Arduino")
        elif command == "glasses":
            self.serial_port.write(b'3')  # Sending 's' to stop all motors
            self.get_logger().info("Sent 'glasses' command to Arduino")
        elif command == "woman":
            self.serial_port.write(b'4')  # Sending 's' to stop all motors
            self.get_logger().info("Sent 'woman' command to Arduino")
        elif command == "tom":
            self.serial_port.write(b'5')  # Sending 's' to stop all motors
            self.get_logger().info("Sent 'tom' command to Arduino")
        elif command == "robert":
            self.serial_port.write(b'6')  # Sending 's' to stop all motors
            self.get_logger().info("Sent 'robert' command to Arduino")
        elif command == "intruder":
            self.serial_port.write(b'i')  # Sending 's' to stop all motors
            self.get_logger().info("Sent 'intruder' command to Arduino")
        else:
            self.get_logger().warn("Invalid command received.")


def main(args=None):
    rclpy.init(args=args)
    arduino_control = arduinoControl()
    rclpy.spin(arduino_control)
    arduino_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()