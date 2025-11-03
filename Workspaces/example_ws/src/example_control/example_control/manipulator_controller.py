#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class ManipulatorController(Node):
  def __init__(self):
    super().__init__("manipulator_controller")
    # Recibr información de posición deseada
    self.end_effector_goal_subscriber = self.create_subscription(
      Twist, "/end_effector_goal", self.end_effector_callback, 10
    )
    # Enviar información de las juntas al controller manager
    self.joint_goals_publisher = self.create_publisher(
      JointState, "/joint_goals", 10
    )
  def end_effector_callback(self, msg:Twist):
    # Valores del efector final -> Valores de las juntas
    # Implementar modelo de cinemática inversa

    # crear un objeto un objeto de robot
    # pasarle los parámetros de dimensiones
    # decirle que plantee la trayectoria
    # decirle que me de las posiciones de las juntas
    pass

def main(args=None):
  try:
    rclpy.init(args=args)
    node = ManipulatorController()
    rclpy.spin(node)
  except KeyboardInterrupt as e:
    print("Node stopped")
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()