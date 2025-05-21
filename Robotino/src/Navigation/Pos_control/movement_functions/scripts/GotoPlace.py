#!/usr/bin/env python
import math
import rospy
import roslib
import math
import tf

roslib.load_manifest('movement_functions')

from geometry_msgs.msg import *
from std_msgs.msg import *

def locations_msg(trans):
    loc_tf = PoseStamped()
    loc_tf.header.stamp = rospy.Time.now()
    loc_tf.header.frame_id = "map"
    loc_tf.pose.position.x, loc_tf.pose.position.y = trans[0],trans[1]
    return loc_tf

def move_to_goal(goal, pub_goal):
    msg_goal = goal
    pub_goal.publish(msg_goal)

def move_distance_angle(goal_dist, goal_angle, pub_goal_dist_angle):
    msg_dist_angle = Float32MultiArray()
    msg_dist_angle.data = [goal_dist, goal_angle]
    pub_goal_dist_angle.publish(msg_dist_angle)

def move_distance(goal_dist, pub_goal_dist):
    msg_dist = Float32()
    msg_dist.data = goal_dist
    pub_goal_dist.publish(msg_dist)


def detect_waveing():
    x,y,z = 3,4,5
    x,y,z = float(x), float(y), float(z)
    p = PointStamped()
    p.header.frame_id = 'camera_link'
    p.point.x, p.point.y, p.point.z = x,y,z
    return p.point

def main():
    print("Go_To_Place Node - - - Running \n")
    rospy.init_node("gotoplace_node", anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(0.1)
    loc_name = '/'+sys.argv[1]
    pub_goal_dist_angle = rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)
    pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, latch=True,queue_size=10)
    print 'Going to: ', loc_name
    trans = 0
    while trans == 0:
        try:
            (trans,rot) = listener.lookupTransform(loc_name, '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    loc_tf = locations_msg(trans)
    print(trans)
    move_to_goal(loc_tf, pub_goal)
    rate.sleep()

if __name__ == "__main__":
    main()