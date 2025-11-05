#!/usr/bin/env python3
from sympy import *

import matplotlib.pyplot as plt

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
    self.xi_0_p = Matrix([
      [x_0_p], 
      [z_0_p], 
      [th_0_p]
    ])
    """print("Matriz de transformación T_0_p: ")
    print(T_0_p.subs([
      (self.theta_0_1, pi/4), (self.theta_1_2, -pi/4), (self.theta_2_3, pi/4)
    ]))"""
    print("vector de postura xi_0_p: ")
    print(self.xi_0_p.subs([
      (self.theta_0_1, pi/4), (self.theta_1_2, -pi/4), (self.theta_2_3, pi/4)
    ]))
  def trajectory_generator(self, q_in = [0.1, 0.1, 0.1], xi_fn = [0, 0, 0], duration = 5):
    self.freq = 10
    print("Definiendo trayectoria")
    # Definiendo polinomio lambda
    # x = xi + lamb() (xf -xi)
    # lam() = a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5
    self.t, a0, a1, a2, a3, a4, a5 = symbols("t, a0, a1, a2, a3, a4, a5")
    self.lam = a0 + a1*self.t + a2*(self.t)**2 + a3*(self.t)**3 + a4*(self.t)**4 + a5*(self.t)**5
    #Primera y segunda derivada
    self.lam_dot = diff(self.lam, self.t)
    self.lam_dot_dot = diff(self.lam_dot, self.t)
    """print(self.lam)
    print(self.lam_dot)
    print(self.lam_dot_dot)"""
    # Cálculo de los parámetros de lambda
    # lam(t=0) = 0
    # lam(t=tf) = 1         ==> lam(t=tf) - 1 = 0
    # lam_dot(t=0) = 0
    # lam_dot(t=tf) = 0
    # lam_dot_dot(t=0) = 0
    # lam_dot_dot(t=tf) = 0
    ec1 = self.lam.subs(self.t, 0)
    ec2 = self.lam.subs(self.t, duration) - 1
    ec3 = self.lam_dot.subs(self.t, 0)
    ec4 = self.lam_dot.subs(self.t, duration)
    ec5 = self.lam_dot_dot.subs(self.t, 0)
    ec6 = self.lam_dot_dot.subs(self.t, duration)
    # Resolver el sistema
    terms = solve([ec1, ec2, ec3, ec4, ec5, ec6], [a0, a1, a2, a3, a4, a5], dict = True)
    self.lam_s          = self.lam.subs(terms[0])
    self.lam_dot_s      = self.lam_dot.subs(terms[0])
    self.lam_dot_dot_s  = self.lam_dot_dot.subs(terms[0])
    """print(self.lam_s)
    print(self.lam_dot_s)
    print(self.lam_dot_dot_s)"""
    # Posición inicial del efector final
    xi_in = self.xi_0_p.subs({
      self.theta_0_1: q_in[0],
      self.theta_1_2: q_in[1],
      self.theta_2_3: q_in[2]
    })
    # Ecuaciones de posición del efector final
    # x_0_p = xi_in[0] + self.lam_s * (xi_fn[0] - xi_in[0])
    xi = xi_in + Matrix([
      [self.lam_s * (xi_fn[0] - xi_in[0])],
      [self.lam_s * (xi_fn[1] - xi_in[1])],
      [self.lam_s * (xi_fn[2] - xi_in[2])]
    ])
    # Ecuaciones de velocidad del efector final
    # x_0_p_dot = self.lam_dot_s * (xi_fn[0] - xi_in[0])
    xi_dot = Matrix([
      [self.lam_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_s * (xi_fn[2] - xi_in[2])]
    ])
    # Ecuaciones de aceleración del efector final
    # x_0_p_dot_dot = self.lam_dot_dot_s * (xi_fn[0] - xi_in[0])
    xi_dot_dot = Matrix([
      [self.lam_dot_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_dot_s * (xi_fn[2] - xi_in[2])]
    ])
    """print("Ecuaciones de posición")
    print("x = {}".format(xi[0]))
    print("Posición final")
    print("x = {}".format(xi[0].subs({self.t: duration})))"""
    # Generar arreglos para muestrear la trayectoria
    self.samples = int(self.freq * duration + 1)
    self.dt = 1/self.freq
    # 3 filas, n columnas (n = número de muestras)
    self.xi_m         = Matrix.zeros(3, self.samples)
    self.xi_dot_m     = Matrix.zeros(3, self.samples)
    self.xi_dot_dot_m = Matrix.zeros(3, self.samples)
    # Arreglo del tiempo
    self.t_m = Matrix.zeros(1, self.samples)
    # Muestreo
    # Tiempo
    self.t_m[0, 0] = 0
    for a in range(self.samples - 1):
      self.t_m[0, a+1] = self.t_m[0, a] + self.dt
    print(self.t_m)
    # Muestreando espacio de trabajo
    # xi_muestreado[columna 1] = xi.substituida(t=algo)
    for a in range(self.samples):
      self.xi_m[:, a]         = xi.subs(self.t, self.t_m[0,a])
      self.xi_dot_m[:, a]     = xi_dot.subs(self.t, self.t_m[0,a])
      self.xi_dot_dot_m[:, a] = xi_dot_dot.subs(self.t, self.t_m[0,a])
    self.simple_graph(self.xi_m, self.t_m)

  def simple_graph(self, val_m, t_m):
    plt.plot(t_m.T, val_m[0, :].T)
    plt.show()



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

    p_x = Matrix([[x],[0],[0]])
    p_y = Matrix([[0],[y],[0]])
    p_z = Matrix([[0],[0],[z]])
    
    
    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_y = Matrix.vstack(Matrix.hstack(R_y, p_y), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_y * T_z

def main():
  robot = RobotKinematics()
  robot.direct_kinematics()
  robot.trajectory_generator()

if __name__ == "__main__":
  main()