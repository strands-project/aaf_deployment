#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os

import rospy
import roslib
import actionlib
from std_msgs.msg import String

import strands_webserver.page_utils as page_utils
import strands_webserver.client_utils as client_utils
from strands_webserver.srv import CallButton, CallButtonResponse
from aaf_walking_group.msg import InterfaceAction, InterfaceResult


class InterfaceServer(object):

    def __init__(self, name):
        # Variables
        self.next_waypoint = ''
        self.request_name = ''
        self._action_name = name
        self.display_no = rospy.get_param("~display", 0)

        # tell the webserver where it should look for web files to serve
        http_root = os.path.join(
            roslib.packages.get_pkg_dir("aaf_walking_group"),
            "www")
        client_utils.set_http_root(http_root)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            InterfaceAction,
            execute_cb=self.executeCallback,
            auto_start=False
        )
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # Services for buttons
        rospy.Service(name+'/button', CallButton, self.button)
        self.pub = rospy.Publisher(name+'/next_waypoint', String, queue_size=10)

    def publishNextWaypoint(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.next_waypoint)
            rate.sleep()

    def executeCallback(self, goal):
        self.request_name = ''
        self.next_waypoint = goal.next_point
        result = InterfaceResult()
        client_utils.display_relative_page(self.display_no, 'guiding.html')

        while self.request_name == '':
            rospy.sleep(0.1)

        if self.request_name == 'next':
            result.chosen_point = goal.next_point
            rospy.loginfo(result)
            # client_utils.display_relative_page(self.display_no,
            #                                    'next_waypoint.html')
            self._as.set_succeeded(result)
        elif self.request_name == 'abort':
            self.preemptCallback()
            # client_utils.display_relative_page(self.display_no, 'abort.html')
        else:
            self.request_name = ''
            self.listWaypointPage(goal.possible_points)
            while self.request_name == '':
                rospy.sleep(0.1)
            result.chosen_point = self.request_name
            rospy.loginfo(result)
            # client_utils.display_relative_page(self.display_no,
            #                                    'next_waypoint.html')
            self._as.set_succeeded(result)

    def listWaypointPage(self, possible_points):
        notice = 'Wahlen Sie Ihren nachsten Punkt.'
        buttons = []
        for i in range(len(possible_points)):
            buttons.append((possible_points[i], 'button'))
        content = page_utils.generate_named_button_page(notice, buttons,
                                                        self._action_name)
        print content
        client_utils.display_content(self.display_no, content)

    def preemptCallback(self):
        rospy.logwarn("Aborting the goal...")
        self._as.set_preempted()

    def button(self, request):
        self.request_name = request.name
        return CallButtonResponse()


if __name__ == '__main__':
    rospy.init_node('interface_server')
    iserver = InterfaceServer(rospy.get_name())
    iserver.publishNextWaypoint()
    rospy.spin()
