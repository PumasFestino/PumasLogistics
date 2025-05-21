#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from pose_estimation.msg import PersonPose3D, Keypoint3D

class Pose3DVisualizer:
    def __init__(self):
        rospy.init_node("pose_3d_visualizer")
        self.sub = rospy.Subscriber("/pose_3d", PersonPose3D, self.pose_callback)
        self.pub = rospy.Publisher("/pose_3d/markers", MarkerArray, queue_size=10)
        self.frame_id = rospy.get_param("~frame_id", "camera_depth_optical_frame")

        rospy.loginfo("Nodo visualizador 3D iniciado")

    def pose_callback(self, msg):
        marker_array = MarkerArray()
        timestamp = rospy.Time.now()

        for i, kp in enumerate(msg.keypoints):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = timestamp
            marker.ns = f"person_{msg.id}"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = kp.x
            marker.pose.position.y = kp.y
            marker.pose.position.z = kp.z
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05  # 5cm
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(marker)

        self.pub.publish(marker_array)

if __name__ == "__main__":
    try:
        Pose3DVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
