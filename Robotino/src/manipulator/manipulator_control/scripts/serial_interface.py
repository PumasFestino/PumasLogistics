#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import serial
import threading

class SerialInterface:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        rospy.Subscriber('manipulator_move', Point, self.queue_move_command)
        rospy.Subscriber('manipulator_gripper', Bool, self.queue_gripper_command)
        rospy.Subscriber('manipulator_home', Bool, self.queue_home_command)  # 🔹 Nuevo tópico para home

        self.command_queue = []
        self.waiting_ok = False

        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        rospy.loginfo("Nodo Serial Bidireccional Iniciado")
        rospy.loginfo("Esperando comandos en: /manipulator_move, /manipulator_gripper y /manipulator_home")

    def queue_move_command(self, msg):
        gcode = "G0"
        if msg.x != 0.0:
            gcode += f" X{msg.x}"
        if msg.y != 0.0:
            gcode += f" Y{msg.y}"
        if msg.z != 0.0:
            gcode += f" Z{msg.z}"
        gcode += "\n"

        self.command_queue.append(gcode)
        rospy.loginfo(f"Comando encolado: {gcode.strip()}")

    def queue_gripper_command(self, msg):
        gripper_pos = 1.0 if msg.data else 0.0
        gcode = f"M3 S{gripper_pos}\n"
        self.command_queue.append(gcode)
        rospy.loginfo(f"Comando encolado: {gcode.strip()}")

    def queue_home_command(self, msg):
        if msg.data:
            gcode = "G28\n"  
            self.command_queue.append(gcode)
            rospy.loginfo("Comando encolado: G28 (Ir a home)")

    def read_serial(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if line:
                    rospy.loginfo(f"ESP32: {line}")
                    if "ok" in line.lower():
                        self.waiting_ok = False

            if not self.waiting_ok and self.command_queue:
                next_command = self.command_queue.pop(0)
                self.ser.write(next_command.encode())
                rospy.loginfo(f"Enviado: {next_command.strip()}")
                self.waiting_ok = True

if __name__ == '__main__':
    rospy.init_node('serial_interface_node')
    interface = SerialInterface()
    rospy.spin()
