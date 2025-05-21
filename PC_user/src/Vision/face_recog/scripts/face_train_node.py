#!/usr/bin/env python

import rospy
import os
import numpy as np
import face_recognition

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2 as cv

class SingleFaceTrainerSaveImageNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Se suscribe a la imagen de entrenamiento
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)

        # Inicializar la lista de codificaciones de la cara
        self.face_encodings = []

        self.train_face = rospy.get_param("~name")

    def image_callback(self, msg):
        try:
            # Convertir la imagen a formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)

            # Encontrar la cara en la imagen
            face_locations = face_recognition.face_locations(cv_image)
            face_encodings = face_recognition.face_encodings(cv_image, face_locations)

            # Agregar la codificacion de la cara a la lista
            self.face_encodings.extend(face_encodings)

            # Guardar la imagen y las codificaciones de la cara en disco
            if len(self.face_encodings) == 1:
                # Guardar la imagen
                cv.imwrite(os.path.expanduser('~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/'+self.train_face+'.jpg'), cv_image)

                # Guardar las codificaciones de la cara en un archivo de texto
                with open(os.path.expanduser('~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Text/'+self.train_face+'.txt'), 'a') as f:
                    np.savetxt(f, self.face_encodings[0])

                rospy.loginfo("Cara entrenada y guardada exitosamente.")
                rospy.signal_shutdown("Cara entrenada y guardada exitosamente.")

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('face_train_node')
    node = SingleFaceTrainerSaveImageNode()
    rospy.spin()
