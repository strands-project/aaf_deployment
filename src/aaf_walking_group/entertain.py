#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import String
import strands_webserver.client_utils


class Entertain(smach.State):
    def __init__(self, display_no):
        smach.State.__init__(
            self,
            outcomes=['key_card'],
            input_keys=['current_waypoint'],
            output_keys=['current_waypoint']
        )
        self.display_no = display_no
        self.sub = rospy.Subscriber("/socialCardReader/commands", String, callback=self.callback)
        self.card = False

    def execute(self, userdata):
        rospy.loginfo("Let me entertain you!")
        strands_webserver.client_utils.display_relative_page(self.display_no, "entertainment.html")
        rospy.loginfo("I am at: " + userdata.current_waypoint)
        while not self.card:
            pass
        return 'key_card'

    def callback(self, data):
        if data.data == "PAUSE_WALK":
            self.card = True
            self.sub.unregister()