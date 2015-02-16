#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os

import rospy
import roslib
import actionlib

import strands_webserver.client_utils as client_utils
from aaf_walking_group.msg import GuidingAction, GuidingResult


class WalkingInterfaceServer(object):

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.display_no = rospy.get_param("~display", 0)

        # tell the webserver where it should look for web files to serve
        http_root = os.path.join(
            roslib.packages.get_pkg_dir("aaf_walking_group"),
            "www")
        client_utils.set_http_root(http_root)

        # Starting server
        rospy.loginfo("%s: Starting walking interface action server", name)
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            GuidingAction,
            execute_cb=self.executeCallback,
            auto_start=False
        )
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

    def executeCallback(self, goal):
        if goal.waypoint == 'right':
            client_utils.display_relative_page(self.display_no,
                                               'turn_right.html')
        elif goal.waypoint == 'left':
            client_utils.display_relative_page(self.display_no,
                                               'turn_left.html')
        else:
            client_utils.display_relative_page(self.display_no,
                                               'straight.html')

        if self._as.is_preempt_requested():
            rospy.logwarn("Aborting the goal...")
            self._as.set_preempted()
        else:
            self._as.set_succeeded(GuidingResult())


if __name__ == '__main__':
    rospy.init_node('walking_interface_server')
    wserver = WalkingInterfaceServer(rospy.get_name())
    rospy.spin()
