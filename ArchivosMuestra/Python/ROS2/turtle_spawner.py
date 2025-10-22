#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')

        # Parámetros configurables
        self.declare_parameter('x', 5.0)
        self.declare_parameter('y', 5.0)
        self.declare_parameter('theta', 0.0)
        self.declare_parameter('name', 'turtle2')

        self.cli = self.create_client(Spawn, '/spawn')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /spawn...')

        self.req = Spawn.Request()
        self.req.x = self.get_parameter('x').value
        self.req.y = self.get_parameter('y').value
        self.req.theta = self.get_parameter('theta').value
        self.req.name = self.get_parameter('name').value

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Tortuga creada: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Error al spawnear tortuga: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()