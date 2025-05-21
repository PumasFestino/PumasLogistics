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

def main():
	path = '/home/pumas/FestinoPumas/PC_user/src/'

	rospy.init_node('festina_startup', anonymous=True)
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	doc_nav_launch 		= roslaunch.parent.ROSLaunchParent(uuid, [path + "Navigation/config_files/launch/late_navigation.launch"])
	main_track_robot_launch = roslaunch.parent.ROSLaunchParent(uuid, [path + "surge_et_ambula/launch/main_track_robot.launch"])

	print("Wait for production")
	
	rospy.sleep(180)
	print("Starting Festina")
	doc_nav_launch.start()
	rospy.sleep(2)
	main_track_robot_launch.start()
	
	try:
	    rospy.spin()
	except KeyboardInterrupt:
		launch.shutdown()
		print("Shutting down")

if __name__ == '__main__':
    main()
