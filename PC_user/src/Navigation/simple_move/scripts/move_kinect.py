#!/usr/bin/env python3
import rospy
import usb.core, usb.util
from simple_move.srv import *

VID, PID = 0x045e, 0x02b0

class KinectMotorNode:
    def __init__(self):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            rospy.logfatal("Kinect motor no encontrado"); exit(1)
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        usb.util.claim_interface(self.dev, 0)

        rospy.Service('kinect/move_tilt', MoveTilt, self.handle_move)
        rospy.Service('kinect/init_tilt', InitTilt, self.handle_init)
        rospy.loginfo("KinectMotorNode listo")

    def handle_init(self, req):
        self.dev.ctrl_transfer(0x40, 0x31, int(0)&0xffff, 0, None, 1000)
        return InitTiltResponse(True)

    def handle_move(self, req):
        rate = rospy.Rate(100)
        timeout = rospy.Time.now() + rospy.Duration(req.time_out)
        angle = (req.angle+int(self.handle_get()/2))
        if -31 <=  angle <= 31:
            while not rospy.is_shutdown() and rospy.Time.now() < timeout:
                self.dev.ctrl_transfer(0x40, 0x31, int(angle*2)&0xffff, 0, None, 1000)
                rate.sleep()
            return MoveTiltResponse(True)
        else:
            return MoveTiltResponse(False)

    def handle_get(self):
        buf = self.dev.ctrl_transfer(0xC0, 0x32, 0, 0, 10)
        if len(buf) != 10:
            return IOError(f"Esperados 10 bytes, recibidos {len(buf)}")
        angle = buf[8] if buf[8] < 128 else buf[8]-256
        status = buf[9]
        return angle

    def cleanup(self):
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)

if __name__=='__main__':
    rospy.init_node('kinect_motor_node')
    node = KinectMotorNode()
    rospy.on_shutdown(node.cleanup)
    rospy.spin()
