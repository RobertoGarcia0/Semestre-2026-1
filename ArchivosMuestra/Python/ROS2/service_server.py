#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceServer(Node):
  def __init__(self):
    super().__init__('simple_service_server')
    server_name = "/sum_two_ints"
    self.srv = self.create_service(AddTwoInts, server_name, self.add_two_ints_callback)
    self.get_logger().info('Servicio disponible en {}'.format(server_name))

  def add_two_ints_callback(self, request:AddTwoInts.Request, response:AddTwoInts.Response):
    response.sum = request.a + request.b
    self.get_logger().info("Recibido: a={}, b={}, respuesta={}".format(request.a, request.b, response.sum))
    return response

def main(args=None):
  rclpy.init(args=args)
  node = SimpleServiceServer()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
