#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from rclpy.task import Future

class SimpleServiceClient(Node):
  def __init__(self):
    super().__init__('simple_service_client')

    service_name = "/sum_two_ints"
    self.a = 10
    self.b = 10

    self.client = self.create_client(AddTwoInts, service_name)
    
    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('Esperando al servicio {}...'.format(service_name))

    request = AddTwoInts.Request()
    request.a = self.a
    request.b = self.b
    # --- Manda solicitud
    future = self.client.call_async(request)
    # --- Función que se ejecuta al recibir respuesta
    future.add_done_callback(self.callback_result)

  def callback_result(self, future:Future):
    try:
      response = future.result()
      response:AddTwoInts.Response
      self.get_logger().info("Resultado recibido: {} + {} = {}".format(self.a, self.b, response.sum))
    except Exception as e:
      self.get_logger().error(f"Error al llamar al servicio: {e}")

def main(args=None):
  rclpy.init(args=args)
  node = SimpleServiceClient()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
