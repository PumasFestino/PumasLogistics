#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from yolo_detect.msg import StringArray
from yolo_detect.msg import PointArray
import logging
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Point  # Nuevo mensaje para el centroide

logging.getLogger('ultralytics').setLevel(logging.WARNING)


def load_model():
    model_path = rospy.get_param('~model_path', '/models/best.pt')
    model = YOLO(model_path)
    rospy.loginfo(f"Loaded YOLOv8 model from {model_path}")
    return model

class YoloCategoryNode:
    def __init__(self):
        self.model = load_model()
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber('/realsense/color/image_raw', Image, self.image_callback)
        self.centroid_pub = rospy.Publisher("/vision/lid_centroid", Point, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")
            return

        results = self.model(cv_image, conf=0.7)[0]

        # Visualización
        for result in results:
            annotated_frame = result.plot()
            cv2.imshow("YOLOv8 lid Estimation", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("Closed by user")

        # Obtener todos los bounding boxes y keypoints
        all_boxes = results.boxes.data.cpu().numpy()
        # print(results.boxes)
        if len(all_boxes) > 0:
            # Encontrar la persona con el bounding box más grande
            # areas = (all_boxes[:, 2] - all_boxes[:, 0]) * (all_boxes[:, 3] - all_boxes[:, 1])
            # selected_idx = areas.argmax()
            
            # Obtener el bounding box y keypoints seleccionados
            # selected_box = all_boxes[selected_idx]
            
            point_array_msg = PointArray()
            for i in range(len(all_boxes)):
                p = Point()
                p.x = float((all_boxes[i][0] + all_boxes[i][2]) / 2)
                p.y = float((all_boxes[i][1] + all_boxes[i][3]) / 2)
                p.z = int(all_boxes[i][5])  # Clase detectada
                point_array_msg.points.append(p)

            self.centroid_pub.publish(point_array_msg)

if __name__ == '__main__':
    rospy.init_node('yolo_lid_node')
    node = YoloCategoryNode()
    rospy.loginfo("YOLO lid node started")
    rospy.spin()