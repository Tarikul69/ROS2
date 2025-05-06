# turtlebot_controller/controller_node.py

#import rclpy
#from rclpy.node import Node
#from geometry_msgs.msg import Twist

#class TurtleBotController(Node):
#    def __init__(self):
#        super().__init__('controller_node')
#        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # ✅ Fixed topic
#        timer_period = 0.5  # seconds
#        self.timer = self.create_timer(timer_period, self.move_forward)

#    def move_forward(self):
#       msg = Twist()
#        msg.linear.x = 0.2   # Move forward
#        msg.angular.z = 0.0  # No rotation
#        self.publisher_.publish(msg)
#        self.get_logger().info('Publishing: Forward')

#def main(args=None):
#    rclpy.init(args=args)
#    node = TurtleBotController()
#    rclpy.spin(node)
#    node.destroy_node()
#    rclpy.shutdown()

#if __name__ == '__main__':
#    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Publisher to control robot velocity
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to LIDAR scan
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Timer to publish movement commands
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.move_robot)

        self.obstacle_detected = False

    def laser_callback(self, msg):
        front_range = msg.ranges[len(msg.ranges) // 2]  # Front-center
        self.obstacle_detected = front_range < 0.5 and front_range > 0.0

    def move_robot(self):
        msg = Twist()
        if self.obstacle_detected:
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Rotate to avoid obstacle
            self.get_logger().info('Obstacle detected! Rotating...')
        else:
            msg.linear.x = 0.2  # Move forward
            msg.angular.z = 0.0
            self.get_logger().info('Moving forward...')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
