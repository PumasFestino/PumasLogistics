#!/usr/bin/env python3
import rospy
import usb.core, usb.util
from sensor_msgs.msg import JointState
from kinect_move.srv import MoveTilt, MoveTiltResponse, InitTilt, InitTiltResponse

VID, PID = 0x045e, 0x02b0

class KinectMotorNode:
    def __init__(self):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            rospy.logfatal("Kinect motor not found")
            exit(1)
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        usb.util.claim_interface(self.dev, 0)

        rospy.Service('kinect/move_tilt', MoveTilt, self.handle_move)
        rospy.Service('kinect/init_tilt', InitTilt, self.handle_init)

        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_name = "kinect_tilt_joint"

        rospy.Timer(rospy.Duration(0.1), self.publish_joint_state)
        rospy.loginfo("KinectMotorNode ready")

    def handle_init(self, req):
        self.dev.ctrl_transfer(0x40, 0x31, 0, 0, None, 1000)
        rospy.loginfo("Kinect tilt initialized")
        return InitTiltResponse(True)

    def handle_move(self, req):
        rate = rospy.Rate(100)
        timeout = rospy.Time.now() + rospy.Duration(req.time_out)
        target_angle = req.angle

        if not -31 <= target_angle <= 31:
            rospy.logwarn("Angle out of range: %.2f", target_angle)
            return MoveTiltResponse(False)

        kinect_angle_val = int(target_angle * 2) & 0xffff

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            try:
                self.dev.ctrl_transfer(0x40, 0x31, kinect_angle_val, 0, None, 1000)
                rate.sleep()
            except usb.core.USBError as e:
                rospy.logerr("Error moving Kinect: %s", e)
                return MoveTiltResponse(False)

        rospy.loginfo("Requested angle reached: %.2f°", target_angle)
        return MoveTiltResponse(True)

    def get_angle(self):
        try:
            buf = self.dev.ctrl_transfer(0xC0, 0x32, 0, 0, 10)
            if len(buf) != 10:
                raise IOError("Expected 10 bytes, received {}".format(len(buf)))
            angle = buf[8] if buf[8] < 128 else buf[8] - 256
            return angle
        except Exception as e:
            rospy.logerr("Error getting angle: %s", e)
            return 0

    def publish_joint_state(self, event):
        angle_deg = self.get_angle()
        angle_rad = angle_deg * 3.1416 / 180.0

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [self.joint_name]
        msg.position = [angle_rad]

        self.joint_pub.publish(msg)

    def cleanup(self):
        try:
            usb.util.release_interface(self.dev, 0)
            usb.util.dispose_resources(self.dev)
            rospy.loginfo("Kinect motor released")
        except Exception as e:
            rospy.logwarn("Error releasing resources: %s", e)

if __name__ == '__main__':
    rospy.init_node('kinect_motor_node')
    node = KinectMotorNode()
    rospy.on_shutdown(node.cleanup)
    rospy.spin()