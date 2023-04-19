# 2023_Bachelor_Project_AGV_Docking
Bachelor project on autonomous docking of an AGV (mobile base: Robotino, cobot:UR5)

Setup Instructions
NOTE: This package has only been tested with ROS Melodic and VMware Workstation 17 Player. It has not been tested with other ROS versions or on a clean Linux machine. Throughout the guide, make sure to follow the instructions for ROS Melodic and Robotino.

Installation
Connect to the school's Robot 2G/5G network.
In the "Edit Virtual Machine Settings" menu, select "Network Adapter" and choose "NAT: used to share the host's IP address".
Check that the network settings on your VM machine are set to "auto".
Run gedit ~/.bashrc and make sure that the ROS_IP and ROS_MASTER_URI lines are commented out or deleted from the bottom of the file.
Install the following packages:
https://github.com/meldew/ros_robotino_rest_pkg
https://github.com/meldew/2023_Bachelor_Project_AGV_Docking
http://wiki.ros.org/ar_track_alvar
Once the packages are installed, start all nodes.
Since Robotino does not have ROS installed, run all packages via Remote PC (i.e., your personal PC, not Robotino's Linux machine).
Camera Calibration
Before running the calibration program, run the following commands (if usb_cam is not already installed, install it now):
Copy code
roscore
rosrun usb_cam usb_cam_node
Run the calibration program:
arduino
Copy code
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam
Once the GUI is open, move a sheet of paper around in front of the webcam at different angles and distances until the "calibrate" button is highlighted. After it finishes calibrating, save and commit the calibration.
Point the camera at your printed AR tags and run the following (replace the marker size and webcam number if needed):
go
Copy code
roslaunch ar_tag_toolbox usb_cam.launch cam_id:=2
roslaunch ar_tag_toolbox ar_track_usb_cam.launch marker_size:=5
Run the following command to see the ID of the AR Tag being detected:
bash
Copy code
rostopic echo /ar_pose_marker
Robot Setup
Connect the camera to Remote PC via USB.
Run the following command to connect to Robotino:
css
Copy code
ssh robotino@172.31.1.145
Start the REST API to Robotino:
Copy code
roslaunch ros_robotino_rest_pkg single_robot_robotino.launch
Start the AR tag tracker:
Copy code
roslaunch ar_tag_toolbox my_ar_track.launch
In Robotino_tag_tracker.py, set target_marker to the ID of your AR tag.
Check if all these packages are installed:
javascript
Copy code
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
