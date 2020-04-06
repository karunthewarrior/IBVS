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
    client = actionlib.SimpleActionClient('align_over_block', AlignOverBlockAction)
    client.wait_for_server()
    goal = AlignOverBlockGoal()
    goal.goal_state.z.data = [0.61] * 4
    goal.goal_state.corners.data = np.array([352, 293, 293, 352, 359, 359, 260, 260])
    # goal.goal_state.corners.data = np.array([352, 293, 293, 359, 359, 260])
    goal.goal_state.execution_mode.data = "DRONE_PICKUP"
    client.send_goal(goal)
    client.wait_for_result()
    r = client.get_result()
    print(r)
 