#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')

        self.declare_parameter('turtle_name', 'turtle1')
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 1.0)

        turtle_name = self.get_parameter('turtle_name').value
        topic = f'/{turtle_name}/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(1, self.move_turtle)

        self.get_logger().info(f"Publicando en {topic}")

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = self.get_parameter('linear_speed').value
        msg.angular.z = self.get_parameter('angular_speed').value
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
