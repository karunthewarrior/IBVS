#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math 
from std_msgs.msg import Int64MultiArray,Float64MultiArray
from mbz2020_ibvs.msg import Int64MultiArrayStamped

class detector:
    def __init__(self):
        img_sub = rospy.Subscriber("/uav1/front_stereo/right/image_color", Image, self.get_img)
        self.z_pub = rospy.Publisher("uav1/z_estimate",Float64MultiArray,queue_size=1)
        self.corners_pub = rospy.Publisher("/uav1/ibvs_corners", Int64MultiArrayStamped, queue_size=1)
        self.bridge = CvBridge()
        self.f = 277.1273
        rospy.sleep(1)

    def get_img(self,img):
        self.img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough"), cv2.COLOR_RGB2BGR)
        self.find_corners()
        self.estimate_z()

    def estimate_z(self):
        img_norm = np.linalg.norm(self.corners - np.roll(self.corners, 1 ,axis=1),axis=0)
        self.z = np.mean(self.f * np.array([0.3,0.3,0.3,0.3])/img_norm) 

    def find_corners(self):
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv, np.array((0, 100, 100)), np.array((10, 255, 255)))
        _,contours,_ = cv2.findContours(self.mask, 1, 2)
        cnt = contours[0]
        rect = cv2.minAreaRect(cnt)
        corners = cv2.boxPoints(rect)
        self.corners = cyclic_sort(np.int0(corners).T)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            out_msg = Int64MultiArrayStamped()
            z_msg = Float64MultiArray()
            out = self.corners
            out_msg.header.stamp = rospy.Time.now()
            out_msg.array.data = out.reshape(-1,)
            z_msg.data = [self.z] * 4
            self.corners_pub.publish(out_msg)
            self.z_pub.publish(z_msg)
            rate.sleep()

def convert_angle(x):
    if x < 0:
        return 360 + x
    else:
        return x
    
def cyclic_sort(pts):
    center = np.mean(pts,axis=1)
    v = pts - center.reshape(-1,1)
    ind = np.argsort([convert_angle(np.rad2deg(math.atan2(y,x))) for x,y in v.T])
    return (pts[:,ind])

if __name__ == "__main__":
    rospy.init_node('detector', anonymous=True)
    m = detector()
    m.run()
