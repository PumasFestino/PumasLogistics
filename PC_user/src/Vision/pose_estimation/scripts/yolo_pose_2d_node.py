#!/usr/bin/env python3
import rospy
import logging
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Point  # Nuevo mensaje para el centroide

from pose_estimation.msg import PersonPose2D, Keypoint2D

logging.getLogger('ultralytics').setLevel(logging.WARNING)

KEYPOINT_NAMES = [
    "nose", "eye_left", "eye_right", "ear_left", "ear_right",
    "shoulder_left", "shoulder_right", "elbow_left", "elbow_right",
    "wrist_left", "wrist_right", "hip_left", "hip_right",
    "knee_left", "knee_right", "ankle_left", "ankle_right"
]

class YoloPoseNode:
    def __init__(self):
        rospy.init_node("yolo_pose_2d_node")
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-pose.pt",verbose=False)  
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("/vision/pose_2d", PersonPose2D, queue_size=10)
        self.centroid_pub = rospy.Publisher("/vision/person_centroid", Point, queue_size=10)

        rospy.loginfo("YOLO Node 2D --- Soft by Joshua M")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"Error to convert image: {e}")
            return
        
    def create_person_message(self, person_id, keypoints):
        person_msg = PersonPose2D()
        person_msg.id = person_id
        person_msg.keypoints = []

        for i, (x, y, conf) in enumerate(keypoints):
            kp_msg = Keypoint2D()
            kp_msg.name = KEYPOINT_NAMES[i]
            kp_msg.x = float(x)
            kp_msg.y = float(y)
            kp_msg.confidence = float(conf)
            person_msg.keypoints.append(kp_msg)

        return person_msg

    def process_image(self, cv_image):
        enabled = rospy.get_param('/pose_2d_enabled', True)
        if not enabled:
            return
        
        results = self.model(cv_image, conf=0.6)[0]
        
        # Visualización
        for result in results:
            annotated_frame = result.plot()
            cv2.imshow("YOLOv8 Pose Estimation", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("Closed by user")

        # Obtener todos los bounding boxes y keypoints
        all_boxes = results.boxes.data.cpu().numpy()
        all_keypoints = results.keypoints.data.cpu().numpy()
        
        if len(all_boxes) > 0:
            # Encontrar la persona con el bounding box más grande
            areas = (all_boxes[:, 2] - all_boxes[:, 0]) * (all_boxes[:, 3] - all_boxes[:, 1])
            selected_idx = areas.argmax()
            
            # Obtener el bounding box y keypoints seleccionados
            selected_box = all_boxes[selected_idx]
            selected_keypoints = all_keypoints[selected_idx]
            
            # Publicar pose de la persona seleccionada
            person_msg = self.create_person_message(selected_idx, selected_keypoints)
            self.pub.publish(person_msg)
            
            # Calcular y publicar centroide
            centroid_x = (selected_box[0] + selected_box[2]) / 2
            centroid_y = (selected_box[1] + selected_box[3]) / 2
            
            centroid_msg = Point()
            centroid_msg.x = float(centroid_x)
            centroid_msg.y = float(centroid_y)
            centroid_msg.z = 0.0
            
            self.centroid_pub.publish(centroid_msg)
            # rospy.loginfo(f"Published centroid for largest person - x: {centroid_x:.1f}, y: {centroid_y:.1f}")

if __name__ == "__main__":
    try:
        YoloPoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass