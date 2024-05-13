import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )

    def laser_callback(self, msg):
        # Extract distances in centimeters and print them
        distances_cm = [round(distance * 100, 2) for distance in msg.ranges]
        print("Distances in cm:", distances_cm)

def main(args=None):
    rclpy.init(args=args)
    laser_subscriber = LaserSubscriber()
    rclpy.spin(laser_subscriber)
    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
