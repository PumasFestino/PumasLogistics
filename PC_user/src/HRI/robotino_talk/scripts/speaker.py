#!/usr/bin/env python
# python

import rospy
import subprocess
from std_msgs.msg import String

def speakcall(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    text = data.data
    subprocess.Popen(["espeak", "-v", "mb-us1","-s", "125", text])

    
def listener():

    rospy.init_node("speaker_node")

    rospy.Subscriber("/speak", String, speakcall)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
