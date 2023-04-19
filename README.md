# 2023_Bachelor_Project_AGV_Docking
Bachelor project on autonomous docking of an AGV (mobile base: Robotino, cobot:UR5)

OPS on:
The package has been tested only with ROS Melodic and VMware Workstation 17 Player. The package has not been tested with other ROS versions or on a clean Linux machine. Throughout the guide, make sure to use the guide created for ROS Melodic and Robotino!

Installation and Usage:
Connect to the school's robot 2G/5G network.
In the virtual machine settings, select:

Go to the Network adapter and select NAT: used to share the host's IP address.
Check if the network settings on your VM machine are set to auto.
Run "gedit ~/.bashrc" and check if the bashrc file contains ROS_IP and ROS_MASTER_URI at the bottom of the file. If you see these, they must be commented out or deleted from the bashrc file.

Installation:
Packages to be installed:

https://github.com/meldew/ros_robotino_rest_pkg
https://github.com/meldew/2023_Bachelor_Project_AGV_Docking
http://wiki.ros.org/ar_track_alvar
Once the packages are installed, you can start all the nodes. Since Robotino does not have ROS installed, you can run all the packages via the Remote PC (i.e., your personal PC, not the Linux machine of Robotino!)

Calibrate the camera with the following steps:
Before running the calibration program, make sure to run the two following commands: (Note: if usb_cam isn't installed already, install it now!)
roscorerosrun usb_cam usb_cam_node
Now run the calibration program:

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam
Once the GUI is open, move your sheet of paper around in front of the webcam, at different angles and distances. Do this until the calibrate button is highlighted and then after it finishes calibrating save and commit it. Now point the camera at your printed AR tags and run the following: (My marker size was 5cm and my webcam was #2 but change if you need)

roslaunch ar_tag_toolbox usb_cam.launch cam_id:=2roslaunch ar_tag_toolbox ar_track_usb_cam.launch marker_size:=5
This next command will let you see the ID of the AR Tag being detected.

rostopic echo /ar_pose_marker

Once the camera is calibrated, do the following:

Connect the camera to the Remote PC via USB.
Then run the following command:
roscore
Connect to Robotino by running:
ssh robotino@172.31.1.145
Start the REST API for Robotino:
roslaunch ros_robotino_rest_pkg single_robot_robotino.launch
Start the AR Tag toolbox:
roslaunch ar_tag_toolbox my_ar_track.launch
In Robotino_tag_tracker.py, set "target_marker" to the ID of your AR tag.
Check if you have all these packages installed:
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import tf.transformations as tf
import numpy as np
import cv_bridge
import cv2 as cv
import rospy
import math
import time

Save the file, place the tag in front of the camera, and run the following command:
5) rosrun ar_tag_toolbox Robotino_tag_tracker.py

It is also possible to control Robotino from the keyboard. If desired, run the following command:
6) rosrun ar_tag_toolbox robot
