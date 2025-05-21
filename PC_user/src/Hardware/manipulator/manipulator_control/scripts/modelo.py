
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

        