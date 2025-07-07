#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point
import numpy as np

class StatePublisher:
    def __init__(self):
        rospy.init_node('state_publisher_node')

        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Tópicos sincronizados con el nodo de interfaz serial
        rospy.Subscriber('manipulator_move', Point, self.position_callback)
        rospy.Subscriber('manipulator_gripper', Bool, self.gripper_callback)
        rospy.Subscriber('manipulator_home', Bool, self.home_callback)

        self.joint_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.target_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        self.speed = [0.03, 0.03, 0.03, 0.02, 0.02]

        self.limits = [
            (0.0, 0.105),    # AxisY_joint
            (-0.18, 0.0),    # AxisZ_joint
            (-0.21592, 0.0), # AxisX_joint
            (0.0, 0.020),    # ClawShovelR_joint
            (0.0, 0.020)     # ClawShovelL_joint
        ]

        self.rate = rospy.Rate(10)
        self.publish_loop()

    def position_callback(self, msg):
        increments = np.array([
            msg.x / 1000.0,
            -msg.y / 1000.0,
            -msg.z / 1000.0
        ])

        for i in range(3):
            proposed_target = self.target_positions[i] + increments[i]
            lower_limit, upper_limit = self.limits[i]

            if proposed_target < lower_limit:
                self.target_positions[i] = lower_limit
            elif proposed_target > upper_limit:
                self.target_positions[i] = upper_limit
            else:
                self.target_positions[i] = proposed_target

    def gripper_callback(self, msg):
        if msg.data:
            self.target_positions[3] = self.limits[3][1]  # ClawShovelR -> abierto
            self.target_positions[4] = self.limits[4][1]  # ClawShovelL -> abierto
        else:
            self.target_positions[3] = self.limits[3][0]  # ClawShovelR -> cerrado
            self.target_positions[4] = self.limits[4][0]  # ClawShovelL -> cerrado

    def home_callback(self, msg):
        if msg.data:
            rospy.loginfo("Regresando a home...")
            self.target_positions[0] = 0.0  # X
            self.target_positions[1] = 0.0  # Y
            self.target_positions[2] = 0.0  # Z

    def publish_loop(self):
        dt = 1.0 / 10.0

        while not rospy.is_shutdown():
            for i in range(5):
                distance = self.target_positions[i] - self.joint_positions[i]
                max_step = self.speed[i] * dt

                if abs(distance) <= max_step:
                    self.joint_positions[i] = self.target_positions[i]
                else:
                    self.joint_positions[i] += max_step * np.sign(distance)

            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['AxisX_joint', 'AxisY_joint', 'AxisZ_joint', 'ClawShovelR_joint', 'ClawShovelL_joint']
            joint_state.position = self.joint_positions.tolist()

            self.joint_pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        StatePublisher()
    except rospy.ROSInterruptException:
        pass
