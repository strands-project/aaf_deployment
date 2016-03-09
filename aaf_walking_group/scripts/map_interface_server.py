#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import roslib
import actionlib
from aaf_walking_group.msg import MapInterfaceAction, MapInterfaceResult
from aaf_walking_group.srv import Button, ButtonResponse
import strands_webserver.client_utils as client_utils

class MapInterfaceServer(object):
    def __init__(self, name):
        # Variables
        self._action_name = name
        self._result = MapInterfaceResult()
        self.display_no = rospy.get_param("~display", 0)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            MapInterfaceAction,
            execute_cb=None,
            auto_start=False
        )
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # ONLY FOR TESTING
        http_root = roslib.packages.get_pkg_dir('aaf_walking_group') + '/www'
        client_utils.set_http_root(http_root)


        # Services for buttons
        rospy.Service(name+'/button', Button, self.button)

    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        client_utils.display_relative_page(self.display_no, 'aaf_map.html')

    def preemptCallback(self):
        self._as.set_preempted()

    def button(self, req):
        self._result.chosen_point = str(req.identifier)
        rospy.loginfo("Selected '%s' via map interface."%self._result)
        self._as.set_succeeded(self._result)
        return ButtonResponse()

if __name__ == '__main__':
    rospy.init_node('map_interface_server')
    MapInterfaceServer(rospy.get_name())
    rospy.spin()
