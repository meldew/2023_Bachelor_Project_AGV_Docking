#!/usr/bin/env python
import subprocess
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf.transformations as tf
#test

def ar_pose_marker_cb(msg):
	n_of_markers_detected = len(msg.markers)
	marker = msg.markers

	if n_of_markers_detected > 0:
		for index in range(n_of_markers_detected):

			 x = marker[index].pose.pose.orientation.x
			 y = marker[index].pose.pose.orientation.y
			 z = marker[index].pose.pose.orientation.z
			 w = marker[index].pose.pose.orientation.w

			 quaterion = [x,y,z,w]
			 Htm = qt2eul_func(quaterion) # Homogeneus transform. matrix


def qt2eul_func(quaterion_matrix):
	R = tf.quaternion_matrix(quaterion_matrix)
	return R

def main():
   	rospy.init_node('qt2eul_node')
   	rospy.Subscriber('/ar_pose_marker',AlvarMarkers, ar_pose_marker_cb)
   	rate = rospy.Rate(10)
   	while not rospy.is_shutdown():

        	rate.sleep()
   	rospy.spin()

if __name__ == '__main__':
  main()
