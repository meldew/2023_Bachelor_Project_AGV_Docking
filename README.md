# 2023_Bachelor_Project_AGV_Docking
Bachelor project on autonomous docking of an AGV (mobile base: Robotino, cobot:UR5)


# Intro video : 

[![Autonomous precision parking with AR](https://user-images.githubusercontent.com/47281451/233119530-1f535c14-d888-4bf5-9a3e-b651063d2868.png)](https://www.youtube.com/watch?v=3h_kH3zjrUo&ab_channel=MikaelWalde "Everything Is AWESOME")

Introduction link.

# Setup Instructions 
`Before Reading:` 
> __Warning__
This package has only been tested with ROS Melodic and VMware Workstation 17 Player. It has not been tested with other ROS versions or on a clean Linux machine. --Throughout the guide, make sure that the following instructions are made for ROS Melodic and Robotino only! The robotic arm guide is `not included!`


 .
 
# Installation and VM setup: 


![image](https://user-images.githubusercontent.com/47281451/233087718-a2e8b6be-58f3-4842-9e0e-6317cbcdca4b.png)

1. Connect to the school's Robot 2G/5G network.
2. In the "Edit Virtual Machine Settings" menu, select "Network Adapter" and choose "NAT: used to share the host's IP address".

![image](https://user-images.githubusercontent.com/47281451/233087765-1ea29109-e8ec-44db-9cb1-c4734efbdca3.png)

3. Check that the network settings on your VM machine are set to "auto".

![image](https://user-images.githubusercontent.com/47281451/233087791-18f94cd3-e51c-4d5c-a45e-c037757fb8a0.png)

4. Run:
```
gedit ~/.bashrc 
```
and make sure that the `ROS_IP` and `ROS_MASTER_URI` lines are commented out or deleted from the bottom of the file.
 
![image](https://user-images.githubusercontent.com/47281451/233087819-5da66fd3-22f6-480d-abeb-9e251a6b521d.png)


5. Install the following packages:

https://github.com/meldew/ros_robotino_rest_pkg

https://github.com/meldew/2023_Bachelor_Project_AGV_Docking

http://wiki.ros.org/ar_track_alvar

6. Since Robotino does not have ROS installed, run all packages via Remote PC (i.e., your personal PC, not Robotino's Linux machine).

# Camera Calibration

1. Before running the calibration program, run the following commands :
```
 roscore
 ```
 
 ```
 rosrun usb_cam usb_cam_node
```
2. Run the calibration program:
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam
```
3. Once the GUI is open, move a sheet of paper around in front of the webcam at different angles and distances until the "calibrate" button is highlighted. After it finishes calibrating, save and then commit the calibration.

![image](https://user-images.githubusercontent.com/47281451/233094555-78f01ae4-601a-4835-b6de-a154c5e1735d.png)


4. Point the camera at your printed AR tags and run the following (replace the marker size and webcam number if needed):

4.1. To create a marker with for exemple id ‘123’ use folowing command: 

```
rosrun ar_track_alvar createMarker 123
```
Then : 
```
roslaunch ar_tag_toolbox usb_cam.launch cam_id:=2
```
You can also run cam_id = 0 as a default if cam_id does not work

```
roslaunch ar_tag_toolbox ar_track_usb_cam.launch marker_size:=5
```
5: Run the following command to see the ID of the AR Tag being detected:
```
rostopic echo /ar_pose_marker
```
# Robot Setup 
1. Connect the camera to `Remote PC` via USB. (`Your own pc, not Robotino's Linux machine`)
2. Run the following command to connect to Robotino:
```
ssh robotino@172.31.1.145
```
3. Start the `REST API` to Robotino:
```
roslaunch ros_robotino_rest_pkg single_robot_robotino.launch
```
4. Start the AR tag tracker:
```
roslaunch ar_tag_toolbox my_ar_track.launch
```
5. In `Robotino_tag_tracker.py`, set target_marker to the `ID` of your `AR tag`.

![image](https://user-images.githubusercontent.com/47281451/233087925-e677eafa-ac74-4c86-aa96-db9d68995528.png)

6. Check if all these packages are installed in `Robotino_tag_tracker.py`:

```python 
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
```
7. Save the file, place the tag in front of the camera, and run the following command:
```
rosrun ar_tag_toolbox Robotino_tag_tracker.py
```

It is also possible to control Robotino from the keyboard. If desired, the following command can be run: 
```
rosrun ar_tag_toolbox robotino_control_tast.py
```
# Cotanct Information
> __Note__ : 
For questions, please write an email to `mi.walde98@gmail.com`.
