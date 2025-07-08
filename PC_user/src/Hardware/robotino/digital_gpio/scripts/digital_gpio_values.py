#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String
from robotino_msgs.msg import DigitalReadings

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_values = rospy.Publisher('set_digital_values', DigitalReadings, queue_size=10)
    rospy.init_node('digital_gpio_values', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        arr_values = DigitalReadings()
        arr_values.stamp.secs = 0
        arr_values.stamp.nsecs = 0
        arr_values.values = [0,0,0,0,0,0]
        pub_values.publish(arr_values)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass