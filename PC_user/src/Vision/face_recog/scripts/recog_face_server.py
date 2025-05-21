#!/usr/bin/env python3

import rospy
import cv2
import os
import numpy as np
import json
from deepface import DeepFace
from datetime import datetime
import tensorflow as tf
from pathlib import Path
import logging
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from face_recog.srv import *

logging.getLogger('tensorflow').setLevel(logging.WARNING)

class DeepFaceRecognitionNode:
    def __init__(self):
        rospy.init_node('deepface_recognition_service')
        
        # Configuración de parámetros
        self.embeddings_path = rospy.get_param('~embeddings_path', '~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Embeddings/')
        self.model_name = rospy.get_param('~model_name', 'Facenet512')
        self.detector_backend = rospy.get_param('~detector_backend', 'retinaface')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.6)
        
        # Configuración de GPU
        self.configure_gpu()
        
        # Expandir y verificar paths
        self.embeddings_path = os.path.expanduser(self.embeddings_path)
        self.verify_directories()
        
        # Inicialización de variables
        self.bridge = CvBridge()
        self.current_image = None
        self.known_face_embeddings = []
        self.known_face_names = []
        self.known_face_metadata = []
        self.database_ready = False
        
        # Cargar caras conocidas
        self.load_known_faces()
        
        # Servicios y suscriptores
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.recognize_face_service = rospy.Service('/vision/recognize_face/names', FaceRecogSrv, self.recognize_face)
        self.result_pub = rospy.Publisher('/face_recognition/deepface_result', String, queue_size=10)
        
        rospy.loginfo(f"Nodo de reconocimiento DeepFace inicializado")

        rospy.loginfo(f"TensorFlow version: {tf.__version__}")
        rospy.loginfo(f"GPU disponible:{tf.config.list_physical_devices('GPU')}")

    def verify_directories(self):
        """Verifica y crea los directorios necesarios"""
        try:
            Path(self.embeddings_path).mkdir(parents=True, exist_ok=True)
            rospy.loginfo(f"Directorio de embeddings verificado: {self.embeddings_path}")
        except Exception as e:
            rospy.logerr(f"No se pudo crear el directorio: {self.embeddings_path}. Error: {e}")

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

    def load_known_faces(self):
        """Carga los embeddings faciales previamente entrenados"""
        start_time = datetime.now()
        loaded_count = 0
        
        try:
            # Verificar si el directorio contiene archivos JSON
            json_files = [f for f in os.listdir(self.embeddings_path) if f.endswith('.json')]
            
            if not json_files:
                rospy.logwarn(f"No se encontraron archivos JSON en {self.embeddings_path}")
                self.database_ready = False
                return
            
            for filename in json_files:
                try:
                    filepath = os.path.join(self.embeddings_path, filename)
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                        
                        if data.get('model') == self.model_name:
                            self.known_face_embeddings.append(np.array(data['embedding']))
                            self.known_face_names.append(data['name'])
                            self.known_face_metadata.append(data['attributes'])
                            loaded_count += 1
                except Exception as e:
                    rospy.logerr(f"Error al cargar {filename}: {e}")
            
            self.database_ready = loaded_count > 0
            rospy.loginfo(f"Cargadas {loaded_count} caras conocidas en {(datetime.now() - start_time).total_seconds():.2f}s")
            
        except Exception as e:
            rospy.logerr(f"Error al cargar caras conocidas: {e}")
            self.database_ready = False

    def image_callback(self, data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Error en CvBridge: {e}")
            return

    def cosine_distance(self, a, b):
        """Calcula la distancia coseno entre dos vectores"""
        return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))

    def recognize_face(self, request):
        response = FaceRecogSrvResponse()
        
        if not request.is_face_recognition_enabled:
            response.names = ["disabled"]
            return response
            
        if self.current_image is None:
            response.names = ["no_image"]
            rospy.logwarn("No hay imagen disponible para reconocimiento")
            return response
            
        if not self.database_ready:
            response.names = ["no_database"]
            rospy.logwarn("Base de datos de caras no está lista")
            return response
            
        try:
            start_time = datetime.now()
            
            # Detectar caras en la imagen
            face_objs = DeepFace.extract_faces(
                img_path=self.current_image,
                detector_backend=self.detector_backend,
                enforce_detection=False,
                align=True
            )
            
            face_names = []
            for face_obj in face_objs:
                if not face_obj['facial_area']:
                    face_names.append("no_face")
                    continue
                
                # Obtener embedding de la cara detectada
                detected_embedding = DeepFace.represent(
                    img_path=self.current_image,
                    model_name=self.model_name,
                    detector_backend=self.detector_backend,
                    enforce_detection=False,
                    align=True
                )[0]['embedding']
                
                # Comparar con caras conocidas
                best_match = None
                best_confidence = 0
                
                for idx, known_embedding in enumerate(self.known_face_embeddings):
                    similarity = self.cosine_distance(detected_embedding, known_embedding)
                    if similarity > best_confidence:
                        best_confidence = similarity
                        best_match = idx
                
                # Determinar si es una coincidencia válida
                if best_confidence >= self.confidence_threshold:
                    name = self.known_face_names[best_match]
                    face_names.append(f"{name}")
                    rospy.loginfo(f"{name} ({best_confidence:.2f})");
                else:
                    face_names.append("unknown")
            
            # Publicar resultados
            self.result_pub.publish(String(",".join(face_names)))
            
            # Preparar respuesta del servicio
            response.names = face_names
            processing_time = (datetime.now() - start_time).total_seconds()
            rospy.loginfo(f"Reconocimiento completado en {processing_time:.2f}s - Resultados: {face_names}")
            
            # Visualización
            # self.visualize_results(self.current_image, face_objs, face_names)
            
        except Exception as e:
            rospy.logerr(f"Error en reconocimiento facial: {e}", exc_info=True)
            response.names = ["error"]
            
        return response

    def visualize_results(self, image, face_objs, names):
        """Visualiza los resultados del reconocimiento"""
        try:
            display_image = image.copy()
            for face_obj, name in zip(face_objs, names):
                if face_obj['facial_area']:
                    x, y, w, h = face_obj['facial_area']['x'], face_obj['facial_area']['y'], \
                                 face_obj['facial_area']['w'], face_obj['facial_area']['h']
                    
                    # Color basado en reconocimiento
                    color = (0, 255, 0) if not name.startswith("unknown") and not name.startswith("no_") else (0, 0, 255)
                    
                    # Dibujar rectángulo
                    cv2.rectangle(display_image, (x, y), (x+w, y+h), color, 2)
                    
                    # Añadir etiqueta
                    cv2.putText(display_image, name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            cv2.imshow('DeepFace Recognition', display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logwarn(f"Error en visualización: {e}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DeepFaceRecognitionNode()
        rospy.loginfo("Servicio de reconocimiento facial con DeepFace listo")
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Apagando nodo de reconocimiento facial")
        cv2.destroyAllWindows()