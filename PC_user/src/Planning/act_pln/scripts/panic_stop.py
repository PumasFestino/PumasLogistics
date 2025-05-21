#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *


def callback_joy(data):
    stop_pub = rospy.Publisher("/navigation/stop",Empty,queue_size=10)
    stop_cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    if(data.buttons[2]):
        alto = Empty()
        stop_pub.publish(alto)
        vel_stop = 0
        stop_cmd_vel_pub.publish(vel_stop)
        #  Node Killer
        """nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            os.system("rosnode kill "+ node)"""
        rospy.sleep(1)


def main():
    rospy.init_node('panic_stop', anonymous=True)
    joy_sub = rospy.Subscriber("/joy",Joy,callback_joy)
    

    rospy.spin()


if __name__ == '__main__':
    main()