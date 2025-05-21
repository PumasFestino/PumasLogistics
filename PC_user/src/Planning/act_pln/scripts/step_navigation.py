#!/usr/bin/env python

import roslaunch
import rospy
import matplotlib
import os
import sys
import tf2_ros
import tf_conversions
import tf
import math
import numpy as np
from geometry_msgs.msg import *
from std_msgs.msg import *
import subprocess

time_over = False
time_start = False


def ejec_script():
        script_shell='/home/pumas/FestinoPumas/PC_user/src/surge_et_ambula/include/map_transfer.sh'
        try:
                print("Sending maps files")
                res=subprocess.call([script_shell],shell=True)
        except:
                print("Error sending the maps")


def callback_time_over(msg):

	global time_over
	global time_start

	if(msg.data == 'start'):
		time_start = True
		print("TS",time_start)
	if(msg.data == "over"):
		time_over = True
		print("TO",time_over)

def main():
	global time_over
	global time_start

	path = '/home/pumas/FestinoPumas/PC_user/src/'

	sub_time_over	= rospy.Subscriber("/time_over",String,callback_time_over)

	tfBuffer = tf2_ros.Buffer()

	rospy.init_node('step_navigation', anonymous=True)
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	#ros_refbox_comm 	= roslaunch.parent.ROSLaunchParent(uuid, [path + "surge_et_ambula/launch/robot_comm.launch"])

	exp_stage_launch 	= roslaunch.parent.ROSLaunchParent(uuid, [path + "surge_et_ambula/launch/exploration_stage.launch"])
	doc_nav_launch 		= roslaunch.parent.ROSLaunchParent(uuid, [path + "Navigation/config_files/launch/navigation.launch"])
	log_zones_launch 	= roslaunch.parent.ROSLaunchParent(uuid, [path + "Navigation/Pos_control/movement_functions/launch/logisticsZones.launch"])
	main_track_robot_launch = roslaunch.parent.ROSLaunchParent(uuid, [path + "surge_et_ambula/launch/main_track_robot.launch"])

	position_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=10)

	listener = tf.TransformListener(1)

	robot_init_pose = PoseWithCovarianceStamped()
	robot_init_pose.header.seq = 1
	robot_init_pose.header.stamp = rospy.Time.now()
	robot_init_pose.header.frame_id = "map"
	
	rospy.sleep(3)

	#ros_refbox_comm.start()
	while(not time_start):
		print("W S",time_start)
		rospy.sleep(2)
		print("Waiting")


	

	while(not time_over):
		rospy.sleep(3)
		print("Starting  Exploration")
		exp_stage_launch.start()
		rospy.loginfo("Robot-Server communication started")

		now = rospy.get_rostime()

		rospy.sleep(180)

		listener.waitForTransform("/base_link", "/map", now, rospy.Duration(4.0))
		(first_trans,first_rot) = listener.lookupTransform("/base_link", "/map", now)
		print("\n Robot a map - SEC POS", first_trans, "\t", first_rot, "at %i", now.secs)

		print("WRITING ORIGIN IN PREV-MAP")
		robot_last_pose.pose.pose.position.x = first_trans[0]
		robot_last_pose.pose.pose.position.y = first_trans[1]
		robot_last_pose.pose.pose.position.z = first_trans[2]
		robot_last_pose.pose.pose.orientation.x = first_rot[0]
		robot_last_pose.pose.pose.orientation.y = first_rot[1]
		robot_last_pose.pose.pose.orientation.z = first_rot[2]
		robot_last_pose.pose.pose.orientation.w = first_rot[3]
		robot_last_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
		position_pub.publish(robot_last_pose)

		rospy.sleep(3)

		now = rospy.Time.now()
		listener.waitForTransform("/base_link", "/map", now, rospy.Duration(4.0))
		(first_trans,first_rot) = listener.lookupTransform("/base_link", "/map", now)
		print("Robot a map - LAST POS", first_trans, "\t", first_rot, "at %i", now.secs)
		
	if(time_over):
		exp_stage_launch.shutdown()
		print("Time over, start Rebecas launch")
		#ejec_script()
		main_track_robot_launch.start()

	try:
	    rospy.spin()
	except KeyboardInterrupt:
		launch.shutdown()
		print("Shutting down")

if __name__ == '__main__':
    main()
