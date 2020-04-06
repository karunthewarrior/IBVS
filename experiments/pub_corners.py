#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from mbz2020_custom_messages.msg import Int32MultiArrayStamped

class corner_detector():
    def __init__(self):
        self.corners_pub = rospy.Publisher("/uav1/ibvs_structure_corners", Int32MultiArrayStamped,queue_size = 1)
        rospy.sleep(1)


    def publish(self):
        out_msg = Int32MultiArrayStamped()
        out_msg.header.stamp = rospy.Time.now()
        out_msg.array.data = np.zeros(4)
        self.corners_pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('corner_publisher')
    detector = corner_detector()
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        detector.publish()
        rate.sleep()
