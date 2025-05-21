#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('pos_definition')
import sys
import rospy
import ros_numpy
import numpy as np
import math
import tf
import tf_conversions
import tf2_ros

from geometry_msgs.msg import *
from std_msgs.msg import *


def move_distance(goal_dist, goal_angle, pub_goal_dist):
  msg_dist = Float32MultiArray()
  msg_dist.data = [goal_dist, goal_angle]
  print(msg_dist)
  pub_goal_dist.publish(msg_dist)

def callback_piece_pos(data):
  global piece_position
  piece_position = [data.point.z, data.point.x,data.point.y]
  print(piece_position)
  br = tf.TransformBroadcaster()
  rate = rospy.Rate(1.0)
  #br.sendTransform((data.point.z, data.point.x, data.point.y), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), "piece_rgbd_sensor_link", "kinect_link")
  br.sendTransform((data.point.z, data.point.x, data.point.y), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), "piece_rgbd_sensor_link", "camera_link")


def main(args):
  rospy.init_node('position_control', anonymous=True)
  print("Position control - Aligning Gripper")
  piece_pos_sub = rospy.Subscriber("/redpiece_pos",PointStamped,callback_piece_pos)
  pub_goal_dist = rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)