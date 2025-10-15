#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
  def __init__(self):
    super().__init__('simple_subscriber')
    #---Parámetros---
    self.declare_parameter("topic_param", "/topic")
    topic_name = self.get_parameter("topic_param").value
    self.subscription = self.create_subscription(
      String,
      topic_name,
      self.listener_callback
    )
    self.get_logger().info('Suscriptor escuchando topico: {}'.format(topic_name))

  def listener_callback(self, msg:String):
    self.get_logger().info('Recibido: {}'.format(msg.data))
    
def main(args=None):
  rclpy.init(args=args)
  node = SimpleSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()