import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math 
from std_msgs.msg import Int64MultiArray, Float64MultiArray
from mbz2020_ibvs.msg import Int64MultiArrayStamped


def gen_interaction(u,v,z):
    L = np.array([[1/z,0,-u/z,-u*v,(1+u**2),-v],[0,1/z,-v/z,-(1+v**2),u*v,u]])
    return L

class servoing:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/uav1/velocity_command', TwistStamped, queue_size=1)
        self.corners_sub = rospy.Subscriber("/uav1/ibvs_corners", Int64MultiArrayStamped, self.get_corners)
        self.z_sub = rospy.Subscriber("/uav1/z_estimate", Float64MultiArray, self.get_z)

        # self.fd = np.array([403, 306, 306, 403, 381, 381, 282, 282]).reshape(2,-1) 

        self.fd = np.array([352, 293, 293, 352, 359, 359, 260, 260]).reshape(2,-1) 
        rospy.sleep(1)

        self.L_star = np.vstack([gen_interaction(u,v,self.z) for (u,v) in self.fd.T])

        rospy.on_shutdown(self.hook)

    def get_corners(self,corners):
        self.f = np.array(corners.array.data).reshape(2,-1)
    
    def get_z(self,z):
        self.z = z.data[0]

    def hook(self):
        v = TwistStamped()
        v.header.frame_id = '/uav1/base_link_stabilized'
        self.vel_pub.publish(v)

    def compute_vel(self):
        L = np.vstack([gen_interaction(u,v,self.z) for (u,v) in self.f.T])
        L = (L + self.L_star)/2
        K = np.array([0.001,0.001,0.1,0,0,1]).reshape(-1,1)
        e = (self.fd - self.f).T.reshape(-1,1)
        x = K * np.dot(np.linalg.pinv(L),e)
        x = np.clip(x,-0.5,0.5)
        return x

    def publish_vel(self,x):
        v = TwistStamped()
        v.header.frame_id = '/uav1/base_link_stabilized'
        v.twist.linear.x = x[1]
        v.twist.linear.y = x[0]
        v.twist.linear.z = x[2]
        v.twist.angular.z = x[-1]
        self.vel_pub.publish(v)

    def run(self):
        if self.f.shape == (2,4):
            x = m.compute_vel()
            self.publish_vel(x)
        else:
            self.publish_vel(np.zeros(6))
            print "No corners"

if __name__ == "__main__":
    rospy.init_node('detector', anonymous=True)
    m = servoing()
    while not rospy.is_shutdown():
        m.run()
