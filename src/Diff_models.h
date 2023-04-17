#include <iostream>
#include <cmath>
#include"C:\\Library\\eigen-3.4.0\\Eigen\\Dense" //path to eigen library, link for sownload .zip https://eigen.tuxfamily.org/index.php?title=Main_Page

namespace diff_model_lib
{
    Eigen::Vector3d omnidirectional_robotino_model_inverse(double vx, double vy, double omega, double d)
    {
        // The function takes in the desired linear velocity in the x and y directions, the desired angular
        // velocity, and the distance between the wheels and the center of the robot, and returns the linear
        // velocities of the three wheels
        
        // :param vx: linear velocity in the x direction
        // :param vy: velocity in the y direction
        // :param omega: angular velocity
        // :param d: distance between the center of the robot and the center of the wheels
        // :return: the velocities of the three wheels.
        
        Eigen::Vector3d Ug(vx, vy ,omega);
        Eigen::Matrix3d Sg;

        Sg <<   -sin(M_PI/3), cos(M_PI/3), d,
                0, -1, d,
                -sin(M_PI/3), cos(M_PI/3), d;

        Eigen::Vector3d Vtot = Sg * Ug;

        double v1 = Vtot(0), v2 = Vtot(1), v3 = Vtot(2);

        return Eigen::Vector3d(v1, v2, v3);
    }

    Eigen::Vector3d omnidirectional_robotino_model_forward(double r, double L, double w1, double w2, double w3, double theta) {
        // The function takes in the robot's wheel radius, length between the center of the robot base and the 
        // center of the wheel, and wheel angular velocities, and
        // outputs the robot's linear and angular velocities in the robot's frame
        
        // :param r: radius of the wheels
        // :param L: distance between the center of the robot and the center of the wheel
        // :param w1: angular velocity of wheel 1
        // :param w2: angular velocity of the middle wheel
        // :param w3: angular velocity of the third wheel
        // :param theta: the orientation of the robot
        // :return: the linear velocity in the x, y, and theta directions.
        
        Eigen::Matrix3d R;
        Eigen::Matrix3d Jinv;
        Eigen::Vector3d V(w1, w2, w3);

        R <<    cos(theta), -sin(theta), 0,
                sin(theta), cos(theta), 0,
                0, 0, 1;

        Jinv << -r/sqrt(3), 0, r/sqrt(3),
                r/3, -2*r/3, r/3,
                r/3*L, r/3*L, r/3*L;


        Eigen::Vector3d lin_vel = R * (Jinv * V);
        double lin_x_vel = lin_vel(0);
        double lin_y_vel = lin_vel(1);
        double lin_theta_vel = lin_vel(2);
        return Eigen::Vector3d(lin_x_vel, lin_y_vel, lin_theta_vel);
    }
}

