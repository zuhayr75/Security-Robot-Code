import rclpy
from robotics.tester import Turtlebot3ObstacleDetection

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3ObstacleDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()





