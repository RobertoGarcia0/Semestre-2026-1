#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose

class SimpleSubscriber(Node):
  def __init__(self):
    super().__init__('simple_subscriber')
    topic_name = "/turtle1/pose"
    self.subscription = self.create_subscription(
      Pose,
      topic_name,
      self.listener_callback,
      10
    )
    self.get_logger().info('Suscriptor escuchando topico: {}'.format(topic_name))

  def listener_callback(self, msg:Pose):
    
    self.get_logger().info("Posición:\nx= {}\ny= {}\ntheta= {}".format(msg.x, msg.y, msg.theta))

def main(args=None):
  rclpy.init(args=args)
  node = SimpleSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()