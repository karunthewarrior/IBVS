# [812.0, 116.0, 888.0, 126.0, 879.0, 202.0, 801.0, 192.0]

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float64
from mbz2020_ibvs.msg import Int64MultiArrayStamped

class corner_converter:
    def __init__(self):
        corner_sub = rospy.Subscriber("/aruco_corners", Float32MultiArray, self.get_corners)
        self.corners_pub = rospy.Publisher("/uav1/ibvs_corners", Int64MultiArrayStamped, queue_size=1)
        self.z_pub = rospy.Publisher("uav1/z_estimate",Float64,queue_size=1)
        rospy.sleep(1)

    def get_corners(self,corners):
        a = corners.data
        self.corners = np.array([[a[0],a[2],a[4],a[6]],[a[1],a[3],a[5],a[7]]])
        
        img_norm = np.linalg.norm(self.corners - np.roll(self.corners, 1 ,axis=1),axis=0)
        z = np.mean(455 * np.array([0.147,0.147,0.147,0.147])/img_norm) 
        z_msg = Float64()
        z_msg.data = z
        self.z_pub.publish(z_msg)
        
        out_msg = Int64MultiArrayStamped()
        out_msg.header.stamp = rospy.Time.now()
        print(self.corners.astype(int))
        out_msg.array.data = self.corners.astype(int).reshape(-1,)
        self.corners_pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node('corner_converter', anonymous=True)
    m = corner_converter()
    rospy.spin()
