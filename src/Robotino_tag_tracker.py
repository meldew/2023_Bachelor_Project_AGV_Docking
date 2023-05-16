#!/usr/bin/env python

# The code is based on a generic approach for controling robots, detecting ar-tags in Python.
# Adapted and implemented by Mikael Walde based on knowledge and examples from various programming resources.
# The controlling part of the code is inspired by the https://github.com/ut-ims-robotics/ar_maze made by Veronika Podliensova and Veiko Vunder.
# All method descriptions are made with AI "Mintlify Doc Writer for Python" extantion for VS Code.

from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import	Image
from nav_msgs.msg import Odometry
import tf.transformations as tf
import numpy as np
import cv_bridge
import cv2 as cv
import rospy
import math
import time

# Creating a new Odometry message and assigning it to the variable Robots_pose_msg.
Robots_pose_msg = Odometry()
# It converts the image from ROS format to OpenCV format.
bridge = cv_bridge.CvBridge()
# Publishing the message to the topic /cmd_vel.
move_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)

#region Global variables 
image = None
angular_z = 0.0
orientation_calibrated = False
distance_to_marker = 0.0
marker_x_position = 0.0
Parked_state = False
marker_is_reached = False
marker_is_detected = False
target_marker = 3 
MAX_ANGULAR_VEL = 0.1
MAX_LINEAR_VEL = 0.03
MAX_LIN_VEL_PARKING = 0.02
PARKING_DISTANCE = 0.19
ANGLE_TOLERANCE = 0.001
X_MARKER_PLACEMENT = 0.003
MAX_CALIBRATING_DISTANCE = 0.3	
e = ""
#endregion

def img_from_camera_cb(img):
	"""
	It converts the image from ROS format to OpenCV format.
	:param img: The image from the camera
	"""
	global image
	image = bridge.imgmsg_to_cv2(img,desired_encoding='bgr8')


def ar_pose_marker_cb(msg):	
	"""
	The function is called every time a new message is received from the topic /ar_pose_marker. The
	function checks if the target marker is detected and if it is, it calculates the distance to the
	marker and the coordinates of the marker in the image. If the marker is detected, the function draws
	a circle around the marker and displays the distance to the marker. If the marker is not detected,
	the function displays a message that the marker is not detected
	
	:param msg: The message that is received from the topic
	"""
	global marker, distance_to_marker,marker_x_position, marker_is_detected, yaw  
	n_of_markers_detected = len(msg.markers)
	marker = msg.markers 
	marker_is_detected = False
	if n_of_markers_detected > 0:
		for index in range(n_of_markers_detected): 

			if marker[index].id == target_marker:
				marker_is_detected = True				
				marker_x_position = marker[index].pose.pose.position.x
				marker_y_position = marker[index].pose.pose.position.y
				distance_to_marker = marker[index].pose.pose.position.z
				tag_pose = msg.markers[index].pose.pose
				tag_orientation = tag_pose.orientation
				roll, yaw, pitch = euler_from_quaternion([-tag_orientation.x, -tag_orientation.y, -tag_orientation.z, tag_orientation.w])

				if marker_is_detected == True and not marker_is_reached and not Parked_state:
					final_x, final_y = ar2cv_coordinate(marker_x_position,marker_y_position,distance_to_marker)
					cv.circle(image,(final_x,final_y),6, (0, 255, 0), -1)
					cv.putText(image, str(round(distance_to_marker * 100)) + " cm to Target!",(final_x + 40,final_y + 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
					cv.putText(image, "Moving to target.",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)

				elif Parked_state == True: 
					cv.putText(image, "Parked!",(300, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
					final_x, final_y = ar2cv_coordinate(marker_x_position,marker_y_position,distance_to_marker)
					cv.circle(image,(final_x,final_y),6, (0, 255, 0), -1)
					cv.putText(image, str(round(distance_to_marker * 100)) + " cm to Target!",(final_x + 40,final_y + 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
				
				else: 
					cv.putText(image, "Searching for a target!.",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)

	cv.imshow('frame', image)
	cv.waitKey(1)
	

def follow_goal_target(): 
	"""
	The robot will move forward until it reaches the AR tag. 
	The robot will turn left or right if it can't find the AR tag. 
	The robot will stop if it reaches the AR tag. 
	The robot will stop if it can't find the AR tag. 
	The robot will stop if the program is interrupted.
	"""
	global marker_is_reached, Parked_state, angular_z
	orientation_calibrated = False
	rate = rospy.Rate(10) 
	last_marker_x_position = []
	try: 
		while not rospy.is_shutdown():
			# The above code is a part of the code that is used to calibrate the robot. The robot is calibrated
			# by moving the robot to a certain distance from the marker and then moving the robot to a certain
			# orientation.
			last_marker_x_position.append(marker_x_position)
			last_marker_x_position = last_marker_x_position[-1:]

			if marker_is_detected == True and distance_to_marker > MAX_CALIBRATING_DISTANCE:
				twist = Twist()
				twist.linear.x = MAX_LINEAR_VEL
				twist.angular.z = -0.6 * math.atan2(marker_x_position, distance_to_marker)
				move_cmd.publish(twist)
				rospy.loginfo(orientation_calibrated)

			elif not orientation_calibrated and distance_to_marker < MAX_CALIBRATING_DISTANCE and marker_is_detected:
				twist = Twist()
				twist.linear.x = 0.0
				twist.linear.y = -marker_x_position * 0.8
				twist.angular.z = -0.2 * yaw
				move_cmd.publish(twist)
				angular_z = twist.angular.z

				if abs(marker_x_position) <= X_MARKER_PLACEMENT and abs(angular_z) <= ANGLE_TOLERANCE:
					orientation_calibrated = True

			elif orientation_calibrated:
				move_robot_forward()

				if distance_to_marker < PARKING_DISTANCE: 
					stop_robot()
					Parked_state = True
					rospy.loginfo("Parked")
					break
					
			# Checking if the marker is not detected and the last marker x position is 0.0. If it is, it will
			# display a message that it is searching for the target and turn the robot.
			elif marker_is_detected != True and last_marker_x_position[-1] == 0.0:
				cv.putText(image, "Searching target",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
				turn_robot()

			# Checking if the marker is not detected and the last marker x position is greater than 0.0. If it
			# is, it will display a message that it is searching for the target and turn the robot right.
			elif marker_is_detected != True and last_marker_x_position[-1] > 0.0:
				cv.putText(image, "Searching target",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
				turn_robot_right()
			# Checking if the marker is not detected and the last marker x position is less than 0.0. If it is,
			# it will display a message that it is searching for the target and turn the robot left.

			elif marker_is_detected != True and last_marker_x_position[-1] < 0.0:
				cv.putText(image, "Searching target",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
				turn_robot_left()

			else: 
				stop_robot()

			rate.sleep()
	except:
		print(e) 
	finally: 
		stop_robot()


def move_robot_forward():
	"""
	It creates a new Twist message, sets the linear velocitie to a slower speed so that robot can safely reach the target and 
	sets angular velocitie to zero, and then publishes the
	message to the move_cmd topic
	"""
	twist = Twist()
	twist.linear.x = MAX_LIN_VEL_PARKING
	twist.angular.z = 0.0
	move_cmd.publish(twist)

def stop_robot():
	"""
	It creates a new Twist message, sets the linear and angular velocities to zero, and publishes the
	message to the move_cmd topic
	"""
	twist = Twist()
	twist.linear.x = 0.0
	twist.angular.z = 0.0
	move_cmd.publish(twist)
	time.sleep(2)


def turn_robot():
	"""
	The function turn_robot() publishes a twist message to the topic /cmd_vel. The twist message has a
	linear velocity of 0 and an angular velocity of 0.1
	"""
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = 0.1
	move_cmd.publish(twist)


def turn_robot_left():
	"""
	It turns the robot left.
	"""
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = MAX_ANGULAR_VEL
	move_cmd.publish(twist)


def turn_robot_right():
	"""
	It creates a Twist message, sets the linear velocity to 0 and the angular velocity to
	-MAX_ANGULAR_VEL, and publishes the message to the move_cmd topic
	"""
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = -MAX_ANGULAR_VEL
	move_cmd.publish(twist)


def map_coordinate(value, in_min, in_max, out_min, out_max):
	"""
	It takes a value, a minimum and maximum value for the input range, and a minimum and maximum value
	for the output range, and returns the value mapped from the input range to the output range
	
	:param value: The value to be mapped
	:param in_min: the minimum value of the input range
	:param in_max: the maximum value of the input range
	:param out_min: The minimum value of the output range
	:param out_max: The maximum value of the output range
	:return: The value of the input mapped to the output range.
	"""
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def qt2eul_func(x,y,z,w):
	"""
	It takes in a quaternion and returns the corresponding homogeneous transformation matrix
	
	:param x: 0.0
	:param y: The yaw angle in radians
	:param z: 0.0
	:param w: 0.0
	:return: the quaternion matrix.
	"""
	quaterion = [x,y,z,w]
	Htm = tf.quaternion_matrix(quaterion)
	return Htm
		

def ar2cv_coordinate(marker_x_position, marker_y_position, z_distance):
	"""
	The function takes the marker's x and y position in the AR coordinate system and the distance from
	the camera to the marker, and returns the marker's x and y position in the OpenCV coordinate system
	
	:param marker_x_position: The x position of the marker in the AR coordinate system
	:param marker_y_position: The y position of the marker in the AR coordinate system
	:param z_distance: The distance from the camera to the marker
	:return: The x and y coordinates of the marker in the image.
	"""
	z_min = 0.20
	z_max = 0.90
	x_min_at_z_min = -0.074
	x_max_at_z_min = 0.080
	y_min_at_z_min = -0.060
	y_max_at_z_min = 0.060
	x_min_at_z_max = -0.34
	x_max_at_z_max = 0.37
	y_min_at_z_max = -0.25
	y_max_at_z_max = 0.25

										
	scale_factor = calculate_scale_factor(z_distance,z_min,z_max)
	values = [x_min_at_z_min + (x_min_at_z_max - x_min_at_z_min), 	
	   		  x_max_at_z_min + (x_max_at_z_max - x_max_at_z_min), 
			  y_min_at_z_min + (y_min_at_z_max - y_min_at_z_min),
			  y_max_at_z_min + (y_max_at_z_max - y_max_at_z_min)]
			  
	x_min = values[0] * scale_factor
	x_max = values[1] * scale_factor
	y_min = values[2] * scale_factor
	y_max = values[3] * scale_factor

	x_mapped = int(round(map_coordinate(marker_x_position, x_min, x_max, 0, 640)))
	y_mapped = int(round(map_coordinate(marker_y_position, y_min, y_max, 0, 480)))

	return x_mapped,y_mapped

def calculate_scale_factor(z_distance,z_min,z_max):
	"""
	The function takes in a z_distance, z_min, and z_max and returns a scale factor
	:param z_distance: The distance from the camera to the object
	:param z_min: The minimum distance from the camera that you want to be able to see
	:param z_max: The maximum distance from the camera that you want to render
	:return: The scale factor is being returned.
	"""
	scale_factor = (z_distance - z_min) / (z_max - z_min)
	return scale_factor

def main():
	# Waiting for the message from the topic /ar_pose_marker.
	rospy.init_node('ar_tag_navigation_node')
	rospy.loginfo("Waiting for ar_pose_marker topic...")
	rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

	
	# Subscribing to the topic /ar_pose_marker and calling the function ar_pose_marker_cb() every time a
	# new message is received.
	rospy.Subscriber('/ar_pose_marker',AlvarMarkers, ar_pose_marker_cb)
	rospy.loginfo("Marker messages detected. Starting follower...")
	rospy.Subscriber('/image_view/output',Image,img_from_camera_cb)
	# Calling the function follow_goal_target()
	follow_goal_target()
	
	
if __name__ == '__main__':
	main()
