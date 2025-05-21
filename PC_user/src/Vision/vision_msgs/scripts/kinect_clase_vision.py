#!/usr/bin/env python
import numpy as np
import cv2
import sys
import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge

def callback_cloud(msg):
    None

def callback_img(msg):
    global frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    print("Image received")

def main():
    global frame
    rospy.init_node("save_frames")
    #rospy.Subscriber("/realsense/depth_registered_points", PointCloud2, callback_cloud)
    rospy.Subscriber("/camera/rgb/image_color", Image, callback_img)

    cap  = cv2.VideoCapture(0)
    counter = 0
    frame = np.zeros((480,640))
    while cap.isOpened() and not rospy.is_shutdown():
        #ret, frame = cap.read()
        # if not ret:
        #     print("Can't receive frame (stream end?). Exiting ...")
        #     break
        cv2.imshow('My Video', frame)
        cmd = cv2.waitKey(50)
        if cmd == ord('s'):
            print("Saving frame")
            cv2.imwrite("video"+str(counter) + ".png", frame)
            counter += 1
        elif cmd == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()