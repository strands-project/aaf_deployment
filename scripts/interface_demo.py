#! /usr/bin/env python

import rospy
import actionlib
from aaf_walking_group.msg import InterfaceAction, InterfaceGoal


class GuideInterfaceTesting(object):

    def __init__(self):

        rospy.loginfo("Creating interface client.")
        self._client = actionlib.SimpleActionClient(
            'interface_server',
            InterfaceAction
        )
        self._client.wait_for_server()
        rospy.loginfo("...done")

    def testPage(self):
        for i in range(4):
            rospy.loginfo("Sending goal...")
            goal = InterfaceGoal()
            goal.next_point = 'test1'
            goal.possible_points = ['test2', 'test3']
            self._client.send_goal_and_wait(goal)
            result = self._client.get_result()

            if result != None:
                result = self._client.get_result()
                rospy.loginfo("Got the result: %s", result.chosen_point)
            else:
                rospy.logwarn("The action was preempted")

            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node("interface_testing")

    page = GuideInterfaceTesting()
    page.testPage()
