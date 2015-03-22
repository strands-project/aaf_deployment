#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import String
import strands_webserver.client_utils


class Entertain(smach.State):
    def __init__(self, display_no):
        smach.State.__init__(
            self,
            outcomes=['key_card', 'killall'],
            input_keys=['current_waypoint'],
            output_keys=['current_waypoint']
        )
        self.display_no = display_no
        self.card = False
        self.sub = None

    def execute(self, userdata):
        self.card = False
        self.sub = rospy.Subscriber("/socialCardReader/commands", String, callback=self.callback)
        rospy.loginfo("Showing entertainment interface.")
        strands_webserver.client_utils.display_relative_page(self.display_no, "entertainment.html")
        rospy.loginfo("I am at: " + userdata.current_waypoint)
        while not self.card and not rospy.is_shutdown() and not self.preempt_requested():
            pass
        if self.preempt_requested():
            self.sub.unregister()
            self.sub = None
            return 'killall'
        return 'key_card'

    def callback(self, data):
        rospy.loginfo("got card: " + str(data.data))
        if data.data == "PAUSE_WALK":
            self.card = True
            self.sub.unregister()
            self.sub = None
