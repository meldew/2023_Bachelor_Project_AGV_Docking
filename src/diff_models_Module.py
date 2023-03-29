from math import sin, cos, pi, sqrt
import numpy as np


# file:///C:/Users/melde/Downloads/Goncalves_ETFA_2008%20(3).pdf
def omnidirectional_robotino_model_inverse(vx, vy, omega,d):
    """
    The function takes in the desired linear velocity in the x and y directions, the desired angular
    velocity, and the distance between the wheels and the center of the robot, and returns the linear
    velocities of the three wheels
    
    :param vx: linear velocity in the x direction
    :param vy: velocity in the y direction
    :param omega: angular velocity
    :param d: distance between the center of the robot and the center of the wheels
    :return: the velocities of the three wheels.
    """
    Ug = np.matrix([[vx],[vy],[omega]]) 

    Sg = np.matrix([[-sin(pi/3), cos(pi/3),d],
                    [0,-1,d],
                    [-sin(pi/3),cos(pi/3),d]])
    
    Vtot = Sg * Ug
    v1, v2 ,v3 = Vtot[0,0],Vtot[1,0],Vtot[2,0]
    return v1, v2 ,v3

# https://www.itm.uni-stuttgart.de/lehre/praktikum-technische-dynamik/PDFfiles/P03_handout.pdf    
def omnidirectional_robotino_model_forward(r, L, w1, w2 ,w3,theta):
    """
    The function takes in the robot's wheel radius, length between the center of the robot base and the 
    center of the wheel, and wheel angular velocities, and
    outputs the robot's linear and angular velocities in the robot's frame
    
    :param r: radius of the wheels
    :param L: distance between the center of the robot and the center of the wheel
    :param w1: angular velocity of wheel 1
    :param w2: angular velocity of the middle wheel
    :param w3: angular velocity of the third wheel
    :param theta: the orientation of the robot
    :return: the linear velocity in the x, y, and theta directions.
    """

    R = np.matrix([[cos(theta), -sin(theta),0],
                   [sin(theta), cos(theta),0],
                   [0,0,1]])
    
    Jinv = np.matrix([[-r/np.sqrt(3), 0, r/np.sqrt(3)],
                      [r/3, -2*r/3, r/3],
                      [r/3*L, r/3*L, r/3*L]])
    
    V = np.matrix([[w1],[w2],[w3]])
    lin_vel = R * (Jinv * V)
    lin_x_vel, lin_y_vel, lin_theta_vel = lin_vel[0,0],lin_vel[1,0],lin_vel[2,0]
    return lin_x_vel, lin_y_vel, lin_theta_vel
