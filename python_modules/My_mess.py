#!/usr/bin/env python
import subprocess
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import	Image
import tf.transformations as tf
import numpy as np
import cv_bridge
import cv2 as cv

bridge = cv_bridge.CvBridge()
image = None

def img_from_camera_cb(img):
	global image
	image = bridge.imgmsg_to_cv2(img,desired_encoding='bgr8')
	
def ar_pose_marker_cb(msg):
	n_of_markers_detected = len(msg.markers)
	marker = msg.markers
	h, w, ch = image.shape #Ehight = 480, Width = 640
	max_x = 0.066
	max_y = 0.066

	camera = np.asmatrix([[1, 0, 0, 0],
                      	  [0, 1, 0, 0],
                      	  [0, 0, 1, 0],
                          [0, 0, 0, 1]])
	
	if n_of_markers_detected > 1:
		marker1_x_position = marker[0].pose.pose.position.x
		marker1_y_position = marker[0].pose.pose.position.y
		marker1_z_position = marker[0].pose.pose.position.z
		x1, y1 = ar2cv2_coordinate(marker1_x_position, marker1_y_position, marker1_z_position)

		marker2_x_position = marker[1].pose.pose.position.x
		marker2_y_position = marker[1].pose.pose.position.y
		marker2_z_position = marker[1].pose.pose.position.z
		x2, y2 = ar2cv2_coordinate(marker2_x_position, marker2_y_position, marker2_z_position) 
		
		distance = np.sqrt(np.power(x2 - x1,2)+ np.power(y2 - y1,2))
		
		cv.line(image, (x1,y1), (x2,y2), (0, 255, 0), 5)
		cv.putText(image, "Distance: " + str(distance),(x1 + 20,y1 + 20), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)

	else:
		print("No points to map")
								  

	if n_of_markers_detected > 0:
		for index in range(n_of_markers_detected):	
			o_x = marker[index].pose.pose.orientation.x
			o_y = marker[index].pose.pose.orientation.y
			o_z = marker[index].pose.pose.orientation.z
			o_w = marker[index].pose.pose.orientation.w
			Htm = qt2eul_func(o_x,o_y,o_z,o_w) # Homogeneus transform. matrix

			marker_x_position = marker[index].pose.pose.position.x
			marker_y_position = marker[index].pose.pose.position.y
			marker_z_position = marker[index].pose.pose.position.z
			

			final_x, final_y = ar2cv2_coordinate(marker_x_position,marker_y_position,marker_z_position)
			#print("x: "+str(final_x)+ "  y: "+str(final_y))

			cv.circle(image,(final_x,final_y),6, (0, 255, 0), -1)
	
		cv.imshow('frame', image)
		cv.waitKey(1)

def map_coordinate(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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
	scale = Calculate_scale_factor(z_distance,z_min,z_max)
	x_min = x_min_at_z_min + (x_min_at_z_max - x_min_at_z_min) * scale
	x_max = x_max_at_z_min + (x_max_at_z_max - x_max_at_z_min) * scale
	y_min = y_min_at_z_min + (y_min_at_z_max - y_min_at_z_min) * scale
	y_max = y_max_at_z_min + (y_max_at_z_max - y_max_at_z_min) * scale

	x_mapped = int(round(map_coordinate(marker_x_position, x_min, x_max, 0, 640)))
	y_mapped = int(round(map_coordinate(marker_y_position, y_min, y_max, 0, 480)))

	return x_mapped,y_mapped

def Calculate_scale_factor(z_distance,z_min,z_max):
    scale_factor = (z_distance - z_min) / (z_max - z_min)
    return scale_factor

def valid_matix(marker,Htm_matrix,camera,index): #(msg.markers , Homogeneus transform. matrix, Matrix for camera, marker index)
	if marker[index].id == 4:
		inverse = np.linalg.inv(np.asmatrix(Htm_matrix))
		ar_3M = inverse * camera
		print(type(inverse))
		print("Cacluclated Homogeneus transform. matrix id X:")
		print(ar_3M)
	elif marker[index].id == 3:
		print("Detected Homogeneus transform. matrix id X:")
		print(Htm_matrix)		

def qt2eul_func(x,y,z,w): # orientation
	quaterion = [x,y,z,w]
	R = tf.quaternion_matrix(quaterion)
	return R

def main():
	rospy.init_node('qt2eul_node')
	rospy.Subscriber('/ar_pose_marker',AlvarMarkers, ar_pose_marker_cb)
	rospy.Subscriber('/image_view/output',Image,img_from_camera_cb)
	

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	main()
