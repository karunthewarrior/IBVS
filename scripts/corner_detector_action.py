#! /usr/bin/env python

import rospy
import numpy as np
import actionlib
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mbz2020_ibvs.msg import Int64MultiArrayStamped
from std_msgs.msg import Int64MultiArray,Float64MultiArray
from mbz2020_ibvs.msg import DetectCornersAction,DetectCornersFeedback,DetectCornersResult

class CornerDetectorServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('corner_detector', DetectCornersAction, self.execute, False)
        self.corners_pub = rospy.Publisher("/uav1/ibvs_corners", Int64MultiArrayStamped,queue_size = 1)

        self.image_sub = rospy.Subscriber("/webcam/camera_stream", Image, self.get_img)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        self.result = DetectCornersResult()
        self.feedback = DetectCornersFeedback()
        self.time_out_limit = 15
        self.time_check_list = []
        rospy.sleep(1)
        self.server.start()

    def get_img(self,im):
        self.img = self.bridge.imgmsg_to_cv2(im)
    
    def detect_corners(self,tag_id):
        corner_msg = Int64MultiArrayStamped()
        try: 
            corners, ids, rejectedImgPoints = aruco.detectMarkers(self.img, self.aruco_dict, parameters=self.parameters)
            ids = [y for x in ids for y in x]
            if tag_id in ids:
                ind = ids.index(tag_id)
                corner = corners[ind]
                a = np.array(corner).flatten()
                self.out_corners = np.array([[a[0],a[2],a[4],a[6]],[a[1],a[3],a[5],a[7]]])
                corner_msg.header.stamp = rospy.Time.now()
                corner_msg.array.data = self.out_corners.astype(int).reshape(-1,)
            return corner_msg
        except:
            rospy.logwarn("Missed Tag Detection")
            return corner_msg
    
    def check_for_corner(self,t):
        if len(self.time_check_list) == 2:
            self.time_check_list = self.time_check_list[:-1]
            self.time_check_list.append(t)
        else:
            self.time_check_list.append(t)

    def execute(self, goal):
        rospy.loginfo("Publishing Corners")
        rate = rospy.Rate(30)
        convergence_time = rospy.Duration(0)
        self.time_check_list = []
        while not rospy.is_shutdown():
            corners = self.detect_corners(goal.tag_id.data)
            # print(corners)
            if len(corners.array.data) != 0:
                self.feedback.corners = corners
                # self.server.publish_feedback(self.feedback)
                self.corners_pub.publish(corners)
                self.time_check_list = []
            else:
                self.check_for_corner(rospy.Time.now())
                if len(self.time_check_list) == 2:
                    convergence_time = self.time_check_list[1] - self.time_check_list[0]
                if (convergence_time > rospy.Duration(self.time_out_limit)):
                    self.result.is_done = False
                    rospy.loginfo("No Corners Found For "+ str(self.time_out_limit)+ " Seconds")
                    self.server.set_succeeded(self.result)
                    break
            if self.server.is_preempt_requested():
                rospy.loginfo("Cancelling action")
                self.result.is_done = False
                self.server.set_preempted(self.result)
                break
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('corner_detector_server')
    server = CornerDetectorServer()
    rospy.spin()
