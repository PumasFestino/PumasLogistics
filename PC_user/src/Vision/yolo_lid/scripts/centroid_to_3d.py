#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np

class CentroidTo3D:
    def __init__(self):
        rospy.init_node('centroid_to_3d_node')
        
        # Suscripciones
        rospy.Subscriber('/vision/lid_centroid', Point, self.centroid_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)
        
        # Publicador
        self.pub_3d = rospy.Publisher('/vision/lid_centroid_3d', Point, queue_size=10)
        
        self.bridge = CvBridge()
        self.current_depth = None
        self.cx = None  # Centroide x (2D)
        self.cy = None  # Centroide y (2D)
        
        # Parámetros intrínsecos (inicializados como None)
        self.fx = None
        self.fy = None
        self.cx_optical = None
        self.cy_optical = None
        
        rospy.loginfo("Node initialized. Waiting for camera info, centroid, and depth data...")

    def camera_info_callback(self, msg):
        """Callback para obtener los parámetros intrínsecos de la cámara."""
        self.fx = msg.K[0]  # fx
        self.fy = msg.K[4]  # fy
        self.cx_optical = msg.K[2]  # cx
        self.cy_optical = msg.K[5]  # cy
        rospy.loginfo_once(f"Camera calibration parameters received: fx={self.fx}, fy={self.fy}, cx={self.cx_optical}, cy={self.cy_optical}")

    def depth_callback(self, msg):
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"Error al procesar depth map: {e}")

    def centroid_callback(self, msg):
        self.cx = msg.x
        self.cy = msg.y
        
        if self.current_depth is not None and all(param is not None for param in [self.fx, self.fy, self.cx_optical, self.cy_optical]):
            try:
                # Obtener profundidad Z (en metros)
                z = self.current_depth[int(self.cy), int(self.cx)] / 1000.0
                
                if np.isnan(z) or z <= 0:
                    rospy.logwarn("Profundidad inválida en el centroide.")
                    return
                
                # Calcular X e Y en 3D
                x = (self.cx - self.cx_optical) * z / self.fx
                y = (self.cy - self.cy_optical) * z / self.fy
                
                # Publicar punto 3D
                point_3d = Point()
                point_3d.x = x
                point_3d.y = y
                point_3d.z = z
                
                self.pub_3d.publish(point_3d)
                rospy.loginfo(f"Centroide 3D publicado: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")
                
            except Exception as e:
                rospy.logerr(f"Error al calcular 3D: {e}")
        else:
            if self.current_depth is None:
                rospy.logwarn_once("Esperando datos de profundidad...")
            if any(param is None for param in [self.fx, self.fy, self.cx_optical, self.cy_optical]):
                rospy.logwarn_once("Esperando parámetros de calibración de la cámara...")

if __name__ == '__main__':
    try:
        node = CentroidTo3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass