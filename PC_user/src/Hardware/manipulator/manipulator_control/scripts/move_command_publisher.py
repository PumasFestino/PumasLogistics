#!/usr/bin/env python

import rospy
from manipulator_control.msg import ManipulatorCommand

def move_command_publisher():
    pub = rospy.Publisher('manipulator_command', ManipulatorCommand, queue_size=10)
    rospy.init_node('move_command_publisher_node', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            x = float(input("Ingrese X (mm): "))
            y = float(input("Ingrese Y (mm): "))
            z = float(input("Ingrese Z (mm): "))
            g = float(input("Ingrese gripper (0 cerrado, 1 abierto): "))

            msg = ManipulatorCommand()
            msg.x = x
            msg.y = y
            msg.z = z
            msg.gripper = g

            pub.publish(msg)
            rospy.loginfo(f"Comando publicado: X={x} Y={y} Z={z} Gripper={g}")

            rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    try:
        move_command_publisher()
    except rospy.ROSInterruptException:
        pass
