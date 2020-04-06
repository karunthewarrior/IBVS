import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

class test:
    def __init__(self):
        self.pose_pub = rospy.Publisher("/uav1/tracking_point", Odometry, queue_size=1)
        #rospy.wait_for_service('/uav1/pose_controller/publish_control')
        #self.call = rospy.ServiceProxy('/uav1/pose_controller/publish_control', SetBool)
        #resp = self.call(False)
        #print(resp)
        rospy.sleep(1)

    def publish_pose(self):
        msg = Odometry()
        msg.header.frame_id = "/uav1/base_link_stabilized"
        msg.child_frame_id = "/uav1/base_link_stabilized"
        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0
        msg.pose.pose.position.z = 5
        # msg.pose.pose.orientation
        self.pose_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('pose_test', anonymous=True)
    d = test()
    d.publish_pose()

    # rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    #     rate.sleep()

    # print ("published")
