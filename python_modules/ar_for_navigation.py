#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import	Image
from nav_msgs.msg import Odometry
import tf.transformations as tf
import numpy as np
import cv_bridge
import cv2 as cv

Robots_odometry_msg = Odometry()
bridge = cv_bridge.CvBridge()
image = None
pub = rospy.Publisher('ar_nav_target', PoseStamped, queue_size=10)
odom_pub = rospy.Publisher('ar_base_odom', Odometry, queue_size=10)

def img_from_camera_cb(img):
	global image
	image = bridge.imgmsg_to_cv2(img,desired_encoding='bgr8')

def ar_pose_marker_cb(msg):	
	n_of_markers_detected = len(msg.markers)
	marker = msg.markers
	target_marker = 3

	world = np.asmatrix([[1, 0, 0, 0],
                      	 [0, 1, 0, 0],
                      	 [0, 0, 1, 0],
                         [0, 0, 0, 1]])

	if n_of_markers_detected > 0:
		for index in range(n_of_markers_detected):
			p_x = marker[index].pose.pose.position.x
			p_y = marker[index].pose.pose.position.y
			p_z = marker[index].pose.pose.position.z

			o_x = marker[index].pose.pose.orientation.x
			o_y = marker[index].pose.pose.orientation.y
			o_z = marker[index].pose.pose.orientation.z
			o_w = marker[index].pose.pose.orientation.w
			ar_Htm = qt2eul_func(o_x,o_y,o_z,o_w)

			ar_Htm[0, 3] = p_x
			ar_Htm[1, 3] = p_y
			ar_Htm[2, 3] = p_z

			if marker[index].id == target_marker:	  
				rospy.loginfo("Target Frame is Detected!")
				navigetion_target = marker[index].pose

				robot_odom = calulate_robot_pose(ar_Htm, world)
				x = robot_odom[0, 3]
				y = robot_odom[1, 3]
				z = robot_odom[2, 3]

				roll, pitch, yaw = tf.euler_from_matrix(robot_odom)
				q_x, q_y, q_z, q_w = tf.quaternion_from_euler(roll,pitch,yaw)
				
				Robots_odometry_msg.header.stamp = rospy.Time.now()
				Robots_odometry_msg.header.frame_id = "odom"
				Robots_odometry_msg.child_frame_id = "base_footprint"

				Robots_odometry_msg.pose.pose.position.x = x
				Robots_odometry_msg.pose.pose.position.y = y
				Robots_odometry_msg.pose.pose.position.z = z
				
				Robots_odometry_msg.pose.pose.orientation.x = q_x 
				Robots_odometry_msg.pose.pose.orientation.y = q_y
				Robots_odometry_msg.pose.pose.orientation.z = q_z
				Robots_odometry_msg.pose.pose.orientation.w = q_w

				pub.publish(navigetion_target)	# Navigation Target
				# This Is not a correct solution for camear to world frame
				#
				# I fix
				#
				odom_pub.publish(Robots_odometry_msg) # Robot Odometry
				#
				#
				#
				#
				marker_x_position = marker[index].pose.pose.position.x
				marker_y_position = marker[index].pose.pose.position.y
				marker_z_position = marker[index].pose.pose.position.z
				
				final_x, final_y = ar2cv2_coordinate(marker_x_position,marker_y_position,marker_z_position)
				cv.circle(image,(final_x,final_y),6, (0, 255, 0), -1)
				cv.putText(image, str(round(marker_z_position * 100)) + " cm to Target!",(final_x + 40,final_y + 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255),2)
	
	cv.imshow('frame', image)
	cv.waitKey(1)


def map_coordinate(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def qt2eul_func(x,y,z,w):
	quaterion = [x,y,z,w]
	Htm = tf.quaternion_matrix(quaterion)
	return Htm

def calulate_robot_pose(Htm_matrix,World): #(Homogeneus transform no translation, Matrix)
	inverse = np.linalg.inv(np.asmatrix(Htm_matrix))
	robot_Pose = inverse * World
	return robot_Pose
		
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

def main():
	rospy.init_node('ar_tag_navigation_node')
	rospy.Subscriber('/ar_pose_marker',AlvarMarkers, ar_pose_marker_cb)
	rospy.Subscriber('/image_view/output',Image,img_from_camera_cb)
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	main()
