#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
  def __init__(self):
    super().__init__('simple_publisher')
    topic_name = "/topic"
    self.publisher = self.create_publisher(String, topic_name, 10)
    self.timer = self.create_timer(1.0, self.timer_callback)
    self.get_logger().info('Publicador iniciado en {}'.format(topic_name))
    self.count = 0

  def timer_callback(self):
    msg = String()
    msg.data = "Mensaje No. {}".format(self.count)
    self.publisher.publish(msg)
    self.get_logger().info('Publicando: {}'.format(msg.data))
    self.count += 1

def main(args=None):
  rclpy.init(args=args)
  node = SimplePublisher()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()