#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from mbz2020_ibvs.msg import AlignOverBlockAction, AlignOverBlockGoal

def shutdown_hook():
    client.cancel_all_goals()
    print ("Shutting down")

if __name__ == '__main__':
    rospy.init_node('align_over_block_client')
    rospy.on_shutdown(shutdown_hook)
    client = actionlib.SimpleActionClient('uav1/align_over_block', AlignOverBlockAction)
    client.wait_for_server()
    goal = AlignOverBlockGoal()
    goal.z.data = [2.36] * 2
    # goal.goal_state.corners.data = np.array([325, 325, 263, 264, 140, 200, 200, 140])
    goal.corners.data = np.array([65,164,212,29])
    goal.execution_mode.data = "DRONE_PLACEMENT"
    client.send_goal(goal)
    client.wait_for_result()
    r = client.get_result()
    print(r)
 
