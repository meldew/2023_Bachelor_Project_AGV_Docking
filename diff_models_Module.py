from math import sin, cos, pi, sqrt
import numpy as np

def omnidirectional_robotino_model_inverse(vx, vy, omega,d):

    # Inverse Kinematics
    Ug = np.matrix([[vx],[vy],[omega]]) 

    Sg = np.matrix([[-sin(pi/3), cos(pi/3),d],
                    [0,-1,d],
                    [-sin(pi/3),cos(pi/3),d]])
    
    Vtot = Sg.dot(Ug)
    return Vtot[0,0],Vtot[1,0],Vtot[2,0]
    
def   omnidirectional_robotino_model_inverse():
    pass   

