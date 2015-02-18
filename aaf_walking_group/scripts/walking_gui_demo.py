#! /usr/bin/env python

import rospy
import actionlib
from aaf_walking_group.msg import GuidingAction, GuidingGoal


class WalkingInterfaceTesting(object):

    def __init__(self):

        rospy.loginfo("Creating interface client.")
        self._client = actionlib.SimpleActionClient(
            'walking_interface_server',
            GuidingAction
        )
        self._client.wait_for_server()
        rospy.loginfo("...done")

    def testPage(self):
        direction = ['right', 'left', 'right', 'left']
        for i in range(len(direction)):
            rospy.loginfo("Sending goal...")
            goal = GuidingGoal()
            goal.waypoint = direction[i]
            self._client.send_goal_and_wait(goal)
            self._client.get_result()
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node("walking_interface_testing")

    page = WalkingInterfaceTesting()
    page.testPage()
