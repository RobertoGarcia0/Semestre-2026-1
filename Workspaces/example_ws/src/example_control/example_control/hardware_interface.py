#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class HardwareInterface(Node):
  def __init__(self):
    super().__init__("hardware_interface")
    # Inicializar las comunicaciones
    # --- 
    # Recibir información de posiciones deseadas
    self.joint_hardware_objectives_subscriber = self.create_subscription(
      JointState, "/joint_hardware_objectives", self.hardware_obj_callback, 10
    )
    # Retroalimentación de las posiciones actuales del robot
    self.joint_states_publisher = self.create_publisher(
      JointState, "/joint_states", 10
    )
    self.create_timer(0.1, self.joint_states_timer_callback)

  def hardware_obj_callback(self, msg:JointState):
    # En esta parte se mandaría a través de alguna comunicación 
    # la información al hardware
    pass

  def joint_states_timer_callback(self):
    msg = JointState()
    # En esta parte recibiría la información del estado del robot
    # ---
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = ["shoulder_joint", "arm_joint", "forearm_joint"]
    msg.position = [0.0, 0.0, 0.0]
    self.joint_states_publisher.publish(msg)

def main(args=None):
  try:
    rclpy.init(args=args)
    node = HardwareInterface()
    rclpy.spin(node)
  except KeyboardInterrupt as e:
    print("Node stopped")
  finally: 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()