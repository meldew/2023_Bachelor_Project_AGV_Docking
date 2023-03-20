#!/usr/bin/env python
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

Robots_pose_msg = Odometry()
bridge = cv_bridge.CvBridge()
move_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)

image = None
distance_to_marker = 0.0
marker_x_position = 0.0
marker_is_reached = False
marker_is_detected = False
target_marker = 3 
MAX_ANGULAR_VEL = 0.1
MAX_LINEAR_VEL = 0.1	
e = ""

def img_from_camera_cb(img):
	global image
	image = bridge.imgmsg_to_cv2(img,desired_encoding='bgr8')

def ar_pose_marker_cb(msg):	
	global marker, distance_to_marker,marker_x_position, marker_is_detected  
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
				if marker_is_detected == True and not marker_is_reached:
					final_x, final_y = ar2cv2_coordinate(marker_x_position,marker_y_position,distance_to_marker)
					cv.circle(image,(final_x,final_y),6, (255, 0, 0), -1)
					cv.putText(image, str(round(distance_to_marker * 100)) + " cm to Target!",(final_x + 40,final_y + 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
					cv.putText(image, "Moving to Target",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
				elif marker_is_reached == True:
					cv.putText(image, "Target is reached",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)

	cv.imshow('frame', image)
	cv.waitKey(1)
	
def follow_goal_target(): 
	global marker_is_reached
	rate = rospy.Rate(10) 
	last_marker_x_position = []
	try: 
		while not rospy.is_shutdown():
			last_marker_x_position.append(marker_x_position)
			last_marker_x_position = last_marker_x_position[-1:]
			if marker_is_detected == True: 
				if distance_to_marker < 0.3: 
					marker_is_reached = True 
					stop_robot()
					rospy.loginfo("Reached AR tag")
					break
				twist = Twist()
				twist.linear.x = MAX_LINEAR_VEL
				twist.angular.z = -0.6 * math.atan2(marker_x_position, distance_to_marker)
				move_cmd.publish(twist)
			elif marker_is_detected != True and last_marker_x_position[-1] == 0.0:
				cv.putText(image, "Searching target",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
				turn_robot()
			elif marker_is_detected != True and last_marker_x_position[-1] > 0.0:
				cv.putText(image, "Searching target",(200, 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
				turn_robot_right()
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

def stop_robot():
	twist = Twist()
	twist.linear.x = 0.0
	twist.angular.z = 0.0
	move_cmd.publish(twist)
	time.sleep(2)

def turn_robot():
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = 0.1
	move_cmd.publish(twist)

def turn_robot_left():
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = MAX_ANGULAR_VEL
	move_cmd.publish(twist)

def turn_robot_right():
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = -MAX_ANGULAR_VEL
	move_cmd.publish(twist)

def map_coordinate(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def qt2eul_func(x,y,z,w):
	quaterion = [x,y,z,w]
	Htm = tf.quaternion_matrix(quaterion)
	return Htm
		
def ar2cv2_coordinate(marker_x_position, marker_y_position, z_distance):
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
	scale = calculate_scale_factor(z_distance,z_min,z_max)
	x_min = x_min_at_z_min + (x_min_at_z_max - x_min_at_z_min) * scale
	x_max = x_max_at_z_min + (x_max_at_z_max - x_max_at_z_min) * scale
	y_min = y_min_at_z_min + (y_min_at_z_max - y_min_at_z_min) * scale
	y_max = y_max_at_z_min + (y_max_at_z_max - y_max_at_z_min) * scale

	x_mapped = int(round(map_coordinate(marker_x_position, x_min, x_max, 0, 640)))
	y_mapped = int(round(map_coordinate(marker_y_position, y_min, y_max, 0, 480)))

	return x_mapped,y_mapped

def calculate_scale_factor(z_distance,z_min,z_max):
    scale_factor = (z_distance - z_min) / (z_max - z_min)
    return scale_factor

def main():
	rospy.init_node('ar_tag_navigation_node')
	rospy.loginfo("Waiting for ar_pose_marker topic...")
	rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
	rospy.Subscriber('/ar_pose_marker',AlvarMarkers, ar_pose_marker_cb)
	rospy.loginfo("Marker messages detected. Starting follower...")
	rospy.Subscriber('/image_view/output',Image,img_from_camera_cb)
	follow_goal_target()
	
	
if __name__ == '__main__':
	main()
