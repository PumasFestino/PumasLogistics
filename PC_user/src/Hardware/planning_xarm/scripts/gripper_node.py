#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from std_msgs.msg import Bool
from dynamixel_sdk import *
from xarm_eef_dynamixel_wrapper.endeffector_port_handler import EndEffectorPortHandler
from xarm.wrapper import XArmAPI
import time

# Configuración Dynamixel
MY_DXL = 'MX-64'
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0
DXL_ID = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 600

# Posiciones del gripper
GRIPPER_MINIMUM_POSITION = 100   # Posición cerrado
GRIPPER_MAXIMUM_POSITION = 3995  # Posición abierto
dxl_goal_positions = [GRIPPER_MINIMUM_POSITION, GRIPPER_MAXIMUM_POSITION]
current_index = 0  # Empieza en posición cerrada

# Configuración xArm
LITE6_IP = "172.27.1.212"

class DynamixelGripperController:
    def __init__(self):
        # Inicializar xArm
        self.arm = XArmAPI(LITE6_IP)
        time.sleep(0.5)
        if self.arm.warn_code != 0:
            self.arm.clean_warn()
        if self.arm.error_code != 0:
            self.arm.clean_error()
        
        #self.arm.motion_enable(True)
        #self.arm.set_mode(0)
        #self.arm.set_state(0)

        # Configurar puerto Dynamixel
        self.portHandler = EndEffectorPortHandler(self.arm)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            rospy.logerr("Failed to open the port")
            rospy.signal_shutdown("Port opening failed")
            return

        if not self.portHandler.setBaudRate(BAUDRATE):
            rospy.logerr("Failed to change the baudrate")
            rospy.signal_shutdown("Baudrate setting failed")
            return

        # Habilitar torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("Dynamixel torque enabled")

        # Suscriptor ROS
        rospy.Subscriber("gripper_control", Bool, self.gripper_callback)
        rospy.loginfo("Gripper controller ready. Waiting for commands...")

    def gripper_callback(self, msg):
        global current_index
        
        # Determinar nueva posición basada en el mensaje
        if msg.data:  # True = abrir
            new_index = 1
            rospy.loginfo("Received OPEN command")
        else:          # False = cerrar
            new_index = 0
            rospy.loginfo("Received CLOSE command")

        # Solo mover si es una nueva posición
        if new_index != current_index:
            self.move_gripper(new_index)
            current_index = new_index

    def move_gripper(self, index):
        goal_pos = dxl_goal_positions[index]
        
        # Escribir posición objetivo
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_pos)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return
        elif dxl_error != 0:
            rospy.logerr("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return

        # Esperar hasta que alcance la posición
        while not rospy.is_shutdown():
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                break
            elif dxl_error != 0:
                rospy.logerr("%s" % self.packetHandler.getRxPacketError(dxl_error))
                break

            rospy.loginfo("GoalPos:%03d  PresPos:%03d" % (goal_pos, dxl_present_position))

            if not abs(goal_pos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                rospy.loginfo("Gripper reached target position")
                break

            time.sleep(0.1)

    def shutdown(self):
        rospy.loginfo("Shutting down gripper controller...")
        
        # Deshabilitar torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        # Cerrar puerto
        self.portHandler.closePort()
        self.arm.disconnect()

if __name__ == '__main__':
    try:
        rospy.init_node('dynamixel_gripper_controller')
        controller = DynamixelGripperController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass