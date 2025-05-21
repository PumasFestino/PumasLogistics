#!/usr/bin/env python

import rospy
import cv2 as cv
import face_recognition
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FaceRecognitionNode:
    def __init__(self):
        rospy.init_node('face_recognition_node')
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/face_recognition/image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.known_face_encodings = []
        self.known_face_names = []
        self.load_known_faces()

    def load_known_faces(self):
        for filename in os.listdir("/home/joshua/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/"):
            name = os.path.splitext(filename)[0]
            print (name)
            image_path = os.path.join("/home/joshua/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/", filename)
            image = face_recognition.load_image_file(image_path)
            face_encoding = face_recognition.face_encodings(image)[0]
            self.known_face_encodings.append(face_encoding)
            self.known_face_names.append(name)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        
        rgb_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_image)
        face_encodings = face_recognition.face_encodings(rgb_image, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            print(matches)
            name = "Unknown"
            if True in matches:
                first_match_index = matches.index(True)
                name = self.known_face_names[first_match_index]
            face_names.append(name)

        print(face_names)

        for (top, right, bottom , left), name in zip(face_locations, face_names):
            cv.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 0), 2)
            cv.putText(cv_image, name, (left, top - 10), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cv.imshow("Image Window", cv_image)
        cv.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    try:
        node = FaceRecognitionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass