#!/usr/bin/env python

import cv2
import numpy as np
from cv2 import aruco
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from mbz2020_ibvs.msg import Int64MultiArrayStamped
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped

class corner_detector():
    def __init__(self):
        self.corners_pub = rospy.Publisher("/uav1/ibvs_corners", Int64MultiArrayStamped, queue_size = 1)
        self.z_pub = rospy.Publisher("/z_estimate", PoseWithCovarianceStamped , queue_size=1)
        self.image_sub = rospy.Subscriber("/webcam/camera_stream", Image, self.get_img)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        self.block_size = 0.1
        self.f = 618.856
        rospy.sleep(1)

    def get_img(self,im):
        self.img = self.bridge.imgmsg_to_cv2(im)

    def estimate_z(self, corners, block_size):
        img_norm = np.linalg.norm(corners - np.roll(corners, 1 ,axis=1),axis=0)
        z = np.mean(self.f * np.array([block_size]*4)/img_norm)
        return z

    def detect_corners(self):
        try: 
            corners, ids, rejectedImgPoints = aruco.detectMarkers(self.img, self.aruco_dict, parameters=self.parameters)
            if len(corners) is not 0:
                a = np.array(corners[0]).flatten()
                self.out_corners = np.array([[a[0],a[2],a[4],a[6]],[a[1],a[3],a[5],a[7]]])
                out_z = PoseWithCovarianceStamped()
                out_z.pose.pose.position.z = self.estimate_z(self.out_corners, self.block_size)
                self.z_pub.publish(out_z)
                out_msg = Int64MultiArrayStamped()
                out_msg.header.stamp = rospy.Time.now()
                out_msg.array.data = self.out_corners.astype(int).reshape(-1,)
                self.corners_pub.publish(out_msg)
        except:
            rospy.logwarn_throttle(2, "Waiting on camera stream")

if __name__ == '__main__':
    rospy.init_node('corner_publisher')
    detector = corner_detector()
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        detector.detect_corners()
        rate.sleep()
