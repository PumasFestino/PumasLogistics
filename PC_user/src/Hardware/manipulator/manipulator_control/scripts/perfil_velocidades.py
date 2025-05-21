#!/usr/bin/env python3

"""
import pybullet as p
import time
import rospkg

# Conectar a la simulación física
physicsClient = p.connect(p.GUI)  # o p.DIRECT para no usar la interfaz gráfica

# Obtener la ruta del paquete
rospack = rospkg.RosPack()
package_path = rospack.get_path('festino_pumas_description')

# Construir la ruta completa al archivo URDF
urdf_path = package_path + "/urdf/festino_pumas.urdf"

# Cargar el URDF
robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1)

# Obtener información de los joints
numJoints = p.getNumJoints(robotId)
jointNames = []
jointIndices = []
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotId, i)
    jointNames.append(jointInfo[1].decode("utf-8"))
    jointIndices.append(jointInfo[0])

# Imprimir nombres de los joints para referencia
print("Joints disponibles:", jointNames)

# Definir las posiciones finales de los joints (ajusta estos valores según tu URDF)
targetPositions = {
    "AxisY_joint": 0.234,  # Posición final para AxisY_joint
    "AxisZ_joint": 0.105,  # Posición final para AxisZ_joint
    "MovingPlate_joint": 0.18  # Posición final para MovingPlate_joint
}

# Mover los joints a sus posiciones finales
for jointName, targetPos in targetPositions.items():
    jointIndex = jointNames.index(jointName)
    p.setJointMotorControl2(robotId, jointIndex, p.POSITION_CONTROL, targetPosition=targetPos)

# Simular el movimiento
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)  # Ajustar el tiempo de simulación

# Desconectar de la simulación
p.disconnect()"""


# import rospy
# from sensor_msgs.msg import JointState

# def move_joints():
#     # Inicializar el nodo de ROS
#     rospy.init_node('mover_joint')

#     # Crear un publicador para el tópico /joint_states
#     pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

#     # Crear un mensaje JointState
#     joint_state = JointState()
#     joint_state.name = ["AxisY_joint", "AxisZ_joint", "MovingPlate_joint"]  # Nombres de los joints
#     joint_state.position = [0.0, 0.0, 0.0]  # Posiciones iniciales de los joints
#     joint_state.velocity = [0.0, 0.0, 0.0]  # Velocidades cero para detener

#     # Publicar las posiciones iniciales para reiniciar los joints
#     for _ in range(10):  # Publicar varias veces para asegurar que se aplique
#         joint_state.header.stamp = rospy.Time.now()
#         pub.publish(joint_state)
#         rospy.sleep(0.1)  # Pequeño retardo entre publicaciones

#     # Posiciones finales de los joints
#     # target_positions = [0.234, 0.105, 0.18]
#     target_positions = [0.334, 0.185, 0.28]

#     # Número de pasos para la interpolación
#     steps = 60  # Menos pasos para mayor velocidad
#     delay = 0.08  # Menor retardo entre pasos

#     # Publicar las posiciones de los joints gradualmente
#     rate = rospy.Rate(1 / delay)  # Frecuencia de publicación
#     for step in range(steps + 1):
#         # Calcular las posiciones interpoladas
#         joint_state.position = [joint_state.position[i] + (target_positions[i] - joint_state.position[i]) / steps for i in range(len(joint_state.position))]
#         joint_state.header.stamp = rospy.Time.now()
#         pub.publish(joint_state)
#         rate.sleep()

#     # Mantener la posición final durante un tiempo adicional
#     hold_time = 5  # Tiempo en segundos para mantener la posición final
#     end_time = rospy.Time.now() + rospy.Duration(hold_time)
#     while rospy.Time.now() < end_time:
#     # while not rospy.is_shutdown():
#         joint_state.header.stamp = rospy.Time.now()
#         pub.publish(joint_state)
#         rate.sleep()

#     # Detener el nodo después de completar el movimiento
#     rospy.signal_shutdown("Movimiento completado")

# if __name__ == '__main__':
#     try:
#         move_joints()
#     except rospy.ROSInterruptException:
#         pass

import rospy
from sensor_msgs.msg import JointState

def stop_robot():
    # Inicializar el nodo de ROS
    rospy.init_node('stop_robot_node')

    # Crear un publicador para el tópico /joint_states
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Crear un mensaje JointState
    joint_state = JointState()
    joint_state.name = ["AxisY_joint", "AxisZ_joint", "MovingPlate_joint"]
    joint_state.position = [0.0, 0.0, 0.0]  # Posiciones cero (o las que quieras)
    joint_state.velocity = [0.0, 0.0, 0.0]  # Velocidades cero para detener

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Publicar continuamente la posición estática
        joint_state.header.stamp = rospy.Time.now()
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        stop_robot()
    except rospy.ROSInterruptException:
        pass