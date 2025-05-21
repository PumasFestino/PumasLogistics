#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from yolo_detect.msg import StringArray
import logging
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

logging.getLogger('ultralytics').setLevel(logging.WARNING)

# Categorías
CATEGORY_MAP = {
    '001_can of fanta': 'Drinks',
    '002_lipton': 'Drinks',
    '003_can of fresca': 'Drinks',
    '004_cocacola': 'Drinks',
    '005_milk': 'Drinks',

    '012_banana': 'Fruits',
    '013_apple': 'Fruits',
    '009_pear': 'Fruits',
    '020_lemon': 'Fruits',

    '010_bali of baseball': 'Toys',
    '021 ball of tenis': 'Toys',
    '022_rubik': 'Toys',
    '017_square': 'Toys',
    '018_lego_4': 'Toys',

    '008_can of tomate': 'Food',
    '014_mustard': 'Food',
    '015_can of tuna': 'Food',
    '016_spam': 'Food',
    '019_tuna': 'Food',
    '011_chips': 'Food',
    '007_cereal': 'Food',

    '023_cup': 'Utensils',
    '024_knife': 'Utensils',
    '025_spoon': 'Utensils',
    '026_plate': 'Utensils',
    '027_bowl': 'Utensils',

    '006_zote': 'Others'
}


def load_model():
    model_path = rospy.get_param('~model_path', '/models/best.pt')
    model = YOLO(model_path)
    rospy.loginfo(f"Loaded YOLOv8 model from {model_path}")
    return model

class YoloCategoryNode:
    def __init__(self):
        self.model = load_model()
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.pub = rospy.Publisher('/detected_objects', StringArray, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")
            return

        results = self.model(cv_image, conf=0.6)[0]
        detections = set()

        for result in results:
            annotated_frame = result.plot()
            cv2.imshow("YOLOv8 Pose Estimation", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("Closed by user")
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                category = CATEGORY_MAP.get(class_name, 'Unknown')
                detections.add(f"{category}: {class_name}")

        detections_list = list(detections)
        rospy.loginfo(f"Detections: {detections_list}")

        msg_out = StringArray()
        msg_out.data = detections_list
        self.pub.publish(msg_out)

if __name__ == '__main__':
    rospy.init_node('yolo_category_node')
    node = YoloCategoryNode()
    rospy.loginfo("YOLO category node started")
    rospy.spin()