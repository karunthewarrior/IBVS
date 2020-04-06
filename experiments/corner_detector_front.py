#!/usr/bin/env python

import cv2
import numpy as np
from cv2 import aruco
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from mbz2020_ibvs.msg import Int64MultiArrayStamped
from std_msgs.msg import Int64MultiArray,Float64MultiArray

class corner_detector():
    def __init__(self):
        self.corners_pub = rospy.Publisher("/uav1/ibvs_corners", Int64MultiArrayStamped,queue_size = 1)
        self.z_pub = rospy.Publisher("uav1/z_estimate",Float64MultiArray,queue_size=1)
        self.image_sub = rospy.Subscriber("/uav1/front_stereo/right/image_color", Image, self.get_img)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()
        self.K = np.array(([277.1273455955935, 0.0, 160.5, 0.0, 277.1273455955935, 120.5, 0.0, 0.0, 1.0])).reshape(3,3)
        self.markerLength = 0.2
        self.z_estimate = None
        rospy.sleep(1)

    def get_img(self,im):
        self.img = self.bridge.imgmsg_to_cv2(im)

    def detect_corners(self):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.img, self.aruco_dict, parameters=self.parameters)
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, self.markerLength, self.K, np.zeros(5))
        
        if(tvecs is not None):
            self.z_estimate = tvecs[0,0,2]

        #rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        if len(corners) is 0:
            self.out_corners = np.array([])
        else:
            a = np.array(corners[0]).flatten()
            self.out_corners = np.array([[a[0],a[2],a[4],a[6]],[a[1],a[3],a[5],a[7]]])
        print(self.out_corners)

    def publish_block_params(self):
        self.detect_corners()
        try:
            img_norm = np.linalg.norm(self.out_corners - np.roll(self.out_corners, 1 ,axis=1),axis=0)

            z = np.mean(253.93 * np.array([0.1778,0.1778,0.1778,0.1778])/img_norm) 
            z_msg = Float64MultiArray()
            z_msg.data = [self.z_estimate] * 4 
            self.z_pub.publish(z_msg)

            out_msg = Int64MultiArrayStamped()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.array.data = self.out_corners.astype(int).reshape(-1,)
            self.corners_pub.publish(out_msg)
            # print out_msg
        except:
            print "Error"

if __name__ == '__main__':
    rospy.init_node('corner_publisher')
    detector = corner_detector()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        detector.publish_block_params()
        rate.sleep()
