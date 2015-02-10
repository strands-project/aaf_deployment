#!/usr/bin/env python

import rospy
import smach


class Entertain(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['key_card'],
            input_keys=['current_waypoint'],
            output_keys=['current_waypoint']
        )

    def execute(self, userdata):
        rospy.loginfo("Let me entertain you!")
        rospy.sleep(1)
        # Entertainment action server
        rospy.loginfo("I am at: " + userdata.current_waypoint)
        return 'key_card'
