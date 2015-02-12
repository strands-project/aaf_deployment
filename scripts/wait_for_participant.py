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
        self._as = actionlib.SimpleActionServer(
            name,
            EmptyAction,
            self.execute,
            auto_start=False
        )
        self._as.start()

    def execute(self, goal):
        print "Called"
        self.pressed = False
        strands_webserver.client_utils.display_relative_page(self.display_no, "continue_page.html")
        while not self.pressed and not rospy.is_shutdown():
            pass
        print "Leave"
        self._as.set_succeeded()

    def button(self, req):
        print "Pressed"
        self.pressed = True
        return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node("wait_for_participant")
    wfp = WaitForParticipant(rospy.get_name())
    rospy.spin()
