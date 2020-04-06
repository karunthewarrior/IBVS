#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from mbz2020_ibvs.msg import DetectCornersAction,DetectCornersGoal

def shutdown_hook():
    client.cancel_all_goals()
    print ("Shutting down")

if __name__ == '__main__':
    rospy.init_node('corner_detector_client')
    rospy.on_shutdown(shutdown_hook)
    client = actionlib.SimpleActionClient('corner_detector', DetectCornersAction)
    client.wait_for_server()
    goal = DetectCornersGoal()
    goal.tag_id.data = 1
    client.send_goal(goal)
    while not rospy.is_shutdown():
        r = client.get_result()
        if r != None:
            print(r, "ONE DONE")
            break
    print("sending second")
    goal = DetectCornersGoal()
    goal.tag_id.data = 2
    client.send_goal(goal)
    while not rospy.is_shutdown():
        r = client.get_result()
        if r != None:
            print(r, "TWO DONE")
            break 
