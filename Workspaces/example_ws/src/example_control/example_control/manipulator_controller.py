#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from .kinematics import RobotKinematics

class ManipulatorController(Node):
  def __init__(self):
    super().__init__("manipulator_controller")
    # Crear un objeto de robot
    self.robot_kinematics = RobotKinematics()
    self.robot_kinematics.direct_kinematics()
    self.end_effector_goal_subscriber = self.create_subscription(
      Twist, "/end_effector_goal", self.end_effector_callback, 10
    )
    # Recibr información de posición actual de las juntas
    self.joint_states_subscriber = self.create_subscription(
      JointState, "/joint_states", self.joint_states_callback, 10
    )
    # Enviar información de las juntas al controller manager
    self.joint_goals_publisher = self.create_publisher(
      JointState, "/joint_goals", 10
    )
    self.get_logger().info("Controlador inicializado")
  def end_effector_callback(self, msg:Twist):
    # Valores del efector final -> Valores de las juntas
    # Decirle que plantee la trayectoria
    self.robot_kinematics.trajectory_generator(self.current_joint_states.position,
                                               [msg.linear.x, msg.linear.z, msg.angular.y], 10)
    # Implementar modelo de cinemática inversa
    self.robot_kinematics.inverse_kinematics()
    # Implementar timer para publicar periódicamente la posición de las juntas
    self.count = 0
    self.joint_goals =  JointState()
    self.joint_goals.name = ["shoulder_joint", "arm_joint", "forearm_joint"]
    self.get_logger().info("Publicando trayectoria de las juntas")
    self.position_publisher_timer = self.create_timer(self.robot_kinematics.dt, 
                                                       self.trayectory_publisher_callback)
  
  def trayectory_publisher_callback(self):
    # Marca de tiempo
    self.joint_goals.header.stamp = self.get_clock().now().to_msg()
    # Obtener valores de las juntas de las matrices de muestreo
    th1 = float(self.robot_kinematics.q_m[0, self.count])
    th2 = float(self.robot_kinematics.q_m[1, self.count])
    th3 = float(self.robot_kinematics.q_m[2, self.count])
    # Asignar valor al mensaje
    self.joint_goals.position = [th1, th2, th3]
    # Publicar
    self.joint_goals_publisher.publish(self.joint_goals)
    self.count+=1
    if (self.count >= len(self.robot_kinematics.q_m[0,:])):
      self.count = 0
      self.position_publisher_timer.cancel()
      self.get_logger().info("Trayectoria finalizada")
    
    
  def joint_states_callback(self, msg:JointState):
    # Recibr información de posición deseada
    self.current_joint_states = msg

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