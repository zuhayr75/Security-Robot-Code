import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import time

class TurtleBotControl(Node):
    def __init__(self):
        super().__init__('turtlebot_control')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Define the linear and angular velocities
        self.linear_speed = 0.2
        self.angular_speed = 0.2

    def move_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(move_cmd)
        time.sleep(2)  # Move forward for 2 seconds

    def turn_right(self):
        move_cmd = Twist()
        move_cmd.angular.z = -self.angular_speed  # Right is negative in ROS
        self.cmd_vel_pub.publish(move_cmd)
        time.sleep(pi / self.angular_speed)  # Turn for pi/2 radians

    def turn_left(self):
        move_cmd = Twist()
        move_cmd.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(move_cmd)
        time.sleep(pi / self.angular_speed)  # Turn for pi/2 radians

    def move_circle(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        move_cmd.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(move_cmd)
        time.sleep(2*pi / self.angular_speed)  # Move in a circle for 2*pi radians

def main(args=None):
    rclpy.init(args=args)
    turtlebot_control = TurtleBotControl()

    try:
        turtlebot_control.move_forward()
        turtlebot_control.turn_right()
        turtlebot_control.move_forward()
        turtlebot_control.turn_left()
        turtlebot_control.move_circle()
    except KeyboardInterrupt:
        pass

    turtlebot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
