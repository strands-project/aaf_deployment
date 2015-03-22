#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_srvs.srv import Empty, EmptyResponse
from aaf_walking_group.msg import EmptyAction
import strands_webserver.client_utils

class WaitForParticipant(object):

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        self.display_no = rospy.get_param("~display_no", 0)
        self.pressed = False
        rospy.Service(name+'/button', Empty, self.button)
        rospy.loginfo("Creating " + name + " server...")
        self._as = actionlib.SimpleActionServer(
            name,
            EmptyAction,
            self.execute,
            auto_start=False
        )
        rospy.loginfo(" ... starting " + name)
        self._as.start()
        rospy.loginfo(" ... started " + name)

    def execute(self, goal):
        print "Called"
        self.pressed = False
        strands_webserver.client_utils.display_relative_page(self.display_no, "continue_page.html")
        while not self.pressed and not rospy.is_shutdown() and not self._as.is_preempt_requested():
            pass
        print "Leave"
        if not self._as.is_preempt_requested():
            self._as.set_succeeded()
        else:
            self._as.set_preempted()

    def button(self, req):
        print "Pressed"
        self.pressed = True
        return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node("wait_for_participant")
    wfp = WaitForParticipant(rospy.get_name())
    rospy.spin()
