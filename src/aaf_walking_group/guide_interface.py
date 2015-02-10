#!/usr/bin/env python

import rospy
import smach


class GuideInterface(smach.State):
    def __init__(self, waypointset):
        smach.State.__init__(
            self,
            outcomes=['move_to_point', 'aborted', 'killall'],
            input_keys=['current_waypoint'],
            output_keys=['waypoint']
        )
        self.waypointset = waypointset

    def execute(self, userdata):
        rospy.loginfo("Showing guide interface")
        rospy.sleep(1)
        # Guide interface returning the next waypoint

        waypoints = self.waypointset['slow']
        rospy.loginfo("I am at: " + userdata.current_waypoint)
        next_waypoint = ""
        for elem in waypoints.items():
            if elem[1] == userdata.current_waypoint:
                key = str(int(elem[0])+1)
                if not key in waypoints.keys():
                    key = "1"
                next_waypoint = waypoints[key]

        rospy.loginfo("I will go to: " + next_waypoint)
        userdata.waypoint = next_waypoint
        return 'move_to_point'
