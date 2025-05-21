#!/usr/bin/env python

import rospy
import os
import numpy as np
from deepface import DeepFace
from datetime import datetime
import tensorflow as tf
import cv2
import json
import logging
from face_recog.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

logging.getLogger('tensorflow').setLevel(logging.WARNING)

class DeepFaceTrainingNode:
    def __init__(self):
        rospy.init_node('deepface_training_node')
        
        # Configuración de parámetros
        self.train_images_path = rospy.get_param('~train_images_path', '~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/')
        self.train_embeddings_path = rospy.get_param('~train_embeddings_path', '~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Embeddings/')
        self.min_faces = rospy.get_param('~min_faces', 1)
        self.max_faces = rospy.get_param('~max_faces', 1)
        self.image_quality = rospy.get_param('~image_quality', 95)
        self.model_name = rospy.get_param('~model_name', 'Facenet512')  # Facenet, VGG-Face, OpenFace, etc.
        
        # Configuración de GPU
        self.configure_gpu()
        
        # Expandir paths de usuario
        self.train_images_path = os.path.expanduser(self.train_images_path)
        self.train_embeddings_path = os.path.expanduser(self.train_embeddings_path)
        
        # Crear directorios si no existen
        os.makedirs(self.train_images_path, exist_ok=True)
        os.makedirs(self.train_embeddings_path, exist_ok=True)
        
        # Inicialización de variables
        self.bridge = CvBridge()
        self.current_image = None
        self.face_data = []
        
        # Servicios y suscriptores
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.train_face_service = rospy.Service('/vision/training_face/name', FaceTrainSrv, self.handle_face_training)
        
        rospy.loginfo(f"Nodo de entrenamiento DeepFace con {self.model_name} inicializado (GPU)")

        rospy.loginfo(f"TensorFlow version: {tf.__version__}")
        rospy.loginfo(f"GPU disponible:{tf.config.list_physical_devices('GPU')}")


    def configure_gpu(self):
        """Configura TensorFlow para usar GPU eficientemente"""
        gpus = tf.config.list_physical_devices('GPU')
        if gpus:
            try:
                for gpu in gpus:
                    tf.config.experimental.set_memory_growth(gpu, True)
                rospy.loginfo("Configuración de GPU completada")
            except RuntimeError as e:
                rospy.logerr(f"Error al configurar GPU: {e}")

    def image_callback(self, data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Error en CvBridge: {e}")
            return

    def handle_face_training(self, req):
        response = FaceTrainSrvResponse()
        
        if self.current_image is None:
            response.success = False
            response.message = "No se ha recibido ninguna imagen de la cámara."
            return response
            
        if not req.name.data.strip():
            response.success = False
            response.message = "El nombre no puede estar vacío."
            return response
            
        try:
            # Detección y extracción de características con DeepFace
            start_time = rospy.Time.now()
            
            # Analizar la imagen con DeepFace
            analysis = DeepFace.analyze(
                img_path=self.current_image,
                actions=['emotion', 'age', 'gender'],
                detector_backend='retinaface',  # Más preciso que 'opencv'
                enforce_detection=True,
                silent=True
            )
            
            # Extraer embeddings faciales
            embeddings = DeepFace.represent(
                img_path=self.current_image,
                model_name=self.model_name,
                detector_backend='retinaface',
                enforce_detection=True
            )
            
            processing_time = (rospy.Time.now() - start_time).to_sec()
            rospy.loginfo(f"Procesamiento DeepFace completado en {processing_time:.2f} segundos")
            
            # Validar número de caras detectadas
            if len(analysis) < self.min_faces:
                response.success = False
                response.message = f"No se detectaron caras. Se requieren al menos {self.min_faces} cara(s)."
                return response
            elif len(analysis) > self.max_faces:
                response.success = False
                response.message = f"Se detectaron {len(analysis)} caras. Solo se permite un máximo de {self.max_faces}."
                return response
                
            # Procesar la cara principal
            main_face = analysis[0]
            main_embedding = embeddings[0]['embedding']
            
            # Crear nombre de archivo único
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename_base = f"{req.name.data}_{timestamp}"
            
            # Guardar imagen con anotaciones
            image_path = os.path.join(self.train_images_path, f"{filename_base}.jpg")
            marked_image = self.annotate_image(self.current_image, main_face)
            cv2.imwrite(image_path, marked_image, [cv2.IMWRITE_JPEG_QUALITY, self.image_quality])
            
            # Guardar metadatos y embeddings en formato JSON
            metadata = {
                'name': req.name.data,
                'timestamp': timestamp,
                'model': self.model_name,
                'embedding': main_embedding,
                'attributes': {
                    'age': main_face['age'],
                    'gender': main_face['gender'],
                    'dominant_emotion': main_face['dominant_emotion'],
                    'emotion': main_face['emotion']
                }
            }
            
            json_path = os.path.join(self.train_embeddings_path, f"{filename_base}.json")
            with open(json_path, 'w') as f:
                json.dump(metadata, f, indent=4)
            
            response.success = True
            response.message = f"Cara de {req.name.data} entrenada exitosamente. Edad: {main_face['age']}, Género: {main_face['gender']}, Emoción: {main_face['dominant_emotion']}"
            rospy.loginfo(f"Entrenamiento exitoso para: {req.name.data}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error en el entrenamiento facial: {str(e)}"
            rospy.logerr(f"Error en DeepFace: {e}", exc_info=True)
            
        return response

    def annotate_image(self, image, face_data):
        """Añade anotaciones a la imagen para visualización"""
        marked_image = image.copy()
        
        # Dibujar rectángulo alrededor del rostro
        x, y, w, h = face_data['region']['x'], face_data['region']['y'], face_data['region']['w'], face_data['region']['h']
        cv2.rectangle(marked_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Añadir texto con información
        info_text = f"{face_data['dominant_emotion']} {face_data['age']} {face_data['gender']}"
        cv2.putText(marked_image, info_text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return marked_image

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DeepFaceTrainingNode()
        rospy.loginfo("Servicio de entrenamiento facial con DeepFace listo")
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Apagando nodo de entrenamiento facial")