# -*- coding: utf-8 -*-
"""
Created on Sat Mar 19 13:44:51 2016

@author: cdondrup
"""

import rospy
import smach
from std_msgs.msg import String
import strands_webserver.client_utils
from aaf_walking_group.srv import Button, ButtonResponse
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest


class SelectRestingChair(smach.State):
    def __init__(self, display_no):
        smach.State.__init__(
            self,
            outcomes=['key_card', 'killall','continue'],
            input_keys=['waypoints', 'play_music'],
            output_keys=['waypoints', 'play_music']
        )
        self.display_no = display_no
        self._result = ''
        rospy.Service("/rast_interface_server/button", Button, self.button)

    def get_node(self, tag):
        try:
            s = rospy.ServiceProxy("/topological_map_manager/get_tagged_nodes", GetTaggedNodes)
            s.wait_for_service()
            return s(GetTaggedNodesRequest(tag=tag)).nodes[0]
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn(e)
            rospy.logwarn("Retrying service call")
            return self.get_node(tag)

    def execute(self, userdata):
        self.card = False
        self.recall_preempt()
        self._result = ''
        sub_key = rospy.Subscriber("/socialCardReader/commands", String, callback=self.key_callback)
        rospy.loginfo("Showing chair selection interface.")
        strands_webserver.client_utils.display_relative_page(self.display_no, userdata.waypoints.get_page())
        while self._result == '' and not rospy.is_shutdown() and not self.preempt_requested():
            rospy.sleep(1)
        sub_key.unregister()
        sub_key = None
        userdata.waypoints.set_resting_chair(self.get_node(self._result))

        if self.preempt_requested():
            if self.card:
                return 'key_card'
            else:
                return 'killall'
        return 'continue'

    def key_callback(self, data):
        rospy.loginfo("got card: " + str(data.data))
        if data.data == "PAUSE_WALK":
            self.card = True
            smach.State.request_preempt(self)

    def button(self, req):
        self._result = str(req.identifier)
        rospy.loginfo("Selected '%s' via map interface."%self._result)
        return ButtonResponse()