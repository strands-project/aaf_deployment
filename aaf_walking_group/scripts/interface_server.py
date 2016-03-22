#! /usr/bin/env python
# -*- coding: utf-8 -*-

# import os

import rospy
# import roslib
import actionlib
from std_msgs.msg import String
from std_srvs.srv import Empty

from aaf_walking_group.msg import MapInterfaceAction, MapInterfaceGoal
import strands_webserver.client_utils as client_utils
from strands_webserver.srv import CallButton, CallButtonResponse
from aaf_walking_group.msg import InterfaceAction, InterfaceResult
from strands_navigation_msgs.srv import GetTaggedNodes


class InterfaceServer(object):
    __resting_node_tag = "walking_group_resting_point"

    def __init__(self, name):
        # Variables
        self.next_waypoint = ''
        self.request_name = ''
        self._action_name = name
        self.display_no = rospy.get_param("~display", 0)

        rospy.loginfo("Creating map interface client...")
        self.client = actionlib.SimpleActionClient(
            '/map_interface_server',
            MapInterfaceAction
        )
        self.client.wait_for_server()
        rospy.loginfo(" ... done ")

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

    def get_closest_tagged_nodes(self):
        s = rospy.ServiceProxy("/topological_localisation/get_nodes_with_tag", GetTaggedNodes)
        s.wait_for_service()
        return s()

    def executeCallback(self, goal):
        self.next_waypoint = goal.next_point
        while not rospy.is_shutdown() and self._as.is_active():
            self.request_name = ''
            result = InterfaceResult()
            client_utils.display_relative_page(self.display_no, 'guiding.html')

            while self.request_name == '' and not rospy.is_shutdown():
                rospy.sleep(0.1)

            if self.request_name == 'next':
                result.chosen_point = self.next_waypoint
                result.idx = goal.idx
                rospy.loginfo(result)
                self._as.set_succeeded(result)
            elif self.request_name == 'abort':
                try:
                    s = rospy.ServiceProxy(
                        '/walking_group/guide_interface/cancel', Empty
                    )
                    s()
                except rospy.ServiceException as e:
                    rospy.logwarn("Service call failed: %s" % e)
                self._as.set_preempted()
            elif self.request_name == 'killall':
                try:
                    s = rospy.ServiceProxy('/walking_group/cancel', Empty)
                    s()
                except rospy.ServiceException as e:
                    rospy.logwarn("Service call failed: %s" % e)
                self._as.set_preempted()
            else:
                self.request_name = ''
                result.chosen_point = ''
                result.idx = self.show_map(goal)
                rospy.loginfo(result)
                if result.chosen_point < 0:
#                    self.request_name = ''
                    continue
                self._as.set_succeeded(result)

    def show_map(self, goal):
        self.client.send_goal_and_wait(MapInterfaceGoal())
        return int(self.client.get_result().chosen_point)

    def preemptCallback(self):
        rospy.logwarn("Aborting the goal...")
        self.request_name = 'abort'
        self.client.cancel_all_goals()

    def button(self, request):
        self.request_name = request.name
        return CallButtonResponse()


if __name__ == '__main__':
    rospy.init_node('interface_server')
    iserver = InterfaceServer(rospy.get_name())
    iserver.publishNextWaypoint()
    rospy.spin()
