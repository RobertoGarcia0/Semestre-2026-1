#!/usr/bin/env python3
from sympy import *
from .kinematics import RobotKinematics

import matplotlib.pyplot as plt

class RobotDynamics():
  def __init__(self):
    pass
  def define_kinematics(self, kinematics:RobotKinematics):
    self.kinematics = kinematics
  def define_dynamics(self, mass = [0.25, 0.25, 0.25]):
    pass

  def lagrange_effort_generator(self):
    pass

  def effort_graph(self):
    fig, ((tau_1_g, tau_2_g, tau_3_g)) = plt.subplots(nrows=1, ncols = 3)
    fig.suptitle("Pares en las juntas")
    # Posiciones ws
    tau_1_g.set_title("Esfuerzo junta 1")
    tau_1_g.plot(self.kinematics.t_m.T, self.tau_m[0, :].T, color = "RED")

    # Velocidades ws
    tau_2_g.set_title("Esfuerzo junta 2")
    tau_2_g.plot(self.kinematics.t_m.T, self.tau_m[1, :].T, color = "GREEN")

    # Aceleraciones ws
    tau_3_g.set_title("Esfuerzo junta 3")
    tau_3_g.plot(self.kinematics.t_m.T, self.tau_m[2, :].T, color = "BLUE")
    plt.show()

  def redefine_kinematics(self):
    self.kinematics = RobotKinematics()
    self.kinematics.direct_kinematics()
    self.kinematics.trajectory_generator(q_in=[0.59, 2.6, 0.25])
    self.kinematics.inverse_kinematics()
  
  def redirect_print(self, new_print):
    global print
    print = new_print

  def inertia_tensor(self, lx, ly, lz, mass):
    return Matrix([[(mass/12.0)*(ly**2 + lz**2), 0, 0], 
                  [0, (mass/12.0)*(lx**2 + lz**2), 0], 
                  [0, 0, (mass/12.0)*(lx**2 + ly**2)]])
  
def main():
  robot = RobotDynamics()
  robot.redefine_kinematics()
  robot.kinematics.ws_graph()
  robot.kinematics.q_graph()
  robot.define_dynamics()
  robot.lagrange_effort_generator()
  robot.effort_graph()

if __name__ == "__main__":
  main()