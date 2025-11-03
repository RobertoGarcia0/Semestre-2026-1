#!/usr/bin/env python3
from sympy import *

class RobotKinematics():
  def __init__(self):
    pass
  def direct_kinematics(self):
    print("Definiendo matrices de transformación")
    self.theta_0_1, self.theta_1_2, self.theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")
    T_0_1 = self.trans_homo(0, 0.1, 0, pi/2, 0, self.theta_0_1)
    T_1_2 = self.trans_homo_xz(0.3, 0, 0, self.theta_1_2)
    T_2_3 = self.trans_homo_xz(0.3, 0, 0, self.theta_2_3)
    T_3_p = self.trans_homo_xz(0.3, 0, 0, 0)
    T_0_p = simplify(T_0_1 * T_1_2 * T_2_3 * T_3_p)

    x_0_p = T_0_p[0, 3]
    z_0_p = T_0_p[2, 3]
    th_0_p = self.theta_0_1 + self.theta_1_2 + self.theta_2_3
    xi_0_p = Matrix([
      [x_0_p], 
      [z_0_p], 
      [th_0_p]
    ])
    print("Matriz de transformación T_0_p: ")
    print(T_0_p.subs([
      (self.theta_0_1, 0), (self.theta_1_2, 0), (self.theta_2_3, 0)
    ]))
    print("vector de postura xi_0_p: ")
    print(xi_0_p.subs([
      (self.theta_0_1, 0), (self.theta_1_2, 0), (self.theta_2_3, 0)
    ]))


  def trans_homo_xz(self, x=0, z=0, gamma=0, alpha=0)->Matrix:
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_z = Matrix([[0],[0],[z]])

    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_z
  
  def trans_homo(self, x, y, z, gamma, beta, alpha):
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_y = Matrix([ [cos(beta), 0, sin(beta)], [0, 1, 0],[-sin(beta), 0, cos(beta)]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    R = R_x * R_y * R_z
    p = Matrix([[x],[y],[z]])
    T = Matrix.vstack(Matrix.hstack(R, p), Matrix([[0,0,0,1]]))
    return T  

def main():
  robot = RobotKinematics()
  robot.direct_kinematics()

if __name__ == "__main__":
  main()