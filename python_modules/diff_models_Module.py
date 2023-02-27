from math import sin, cos, pi, sqrt
import numpy as np


# file:///C:/Users/melde/Downloads/Goncalves_ETFA_2008%20(3).pdf
def omnidirectional_robotino_model_inverse(vx, vy, omega,d):
    Ug = np.matrix([[vx],[vy],[omega]]) 

    Sg = np.matrix([[-sin(pi/3), cos(pi/3),d],
                    [0,-1,d],
                    [-sin(pi/3),cos(pi/3),d]])
    
    Vtot = Sg * Ug
    return Vtot[0,0],Vtot[1,0],Vtot[2,0]

# https://www.itm.uni-stuttgart.de/lehre/praktikum-technische-dynamik/PDFfiles/P03_handout.pdf    
def omnidirectional_robotino_model_forward(d, L, w1, w2 ,w3,theta):
    R = np.matrix([[cos(theta), -sin(theta),0],
                   [sin(theta), cos(theta),0],
                   [0,0,1]])
    
    Jinv = np.matrix([[-d/np.sqrt(3), 0, d/np.sqrt(3)],
                      [d/3, -2*d/3, d/3],
                      [d/3*L, d/3*L, d/3*L]])
    
    V = np.matrix([[w1],[w2],[w3]])
    lin_vel = R * (Jinv * V)
    lin_x_vel, lin_y_vel, lin_theta_vel = lin_vel[0,0],lin_vel[1,0],lin_vel[2,0]
    return lin_x_vel, lin_y_vel, lin_theta_vel
