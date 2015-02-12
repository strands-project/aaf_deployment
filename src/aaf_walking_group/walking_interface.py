#!/usr/bin/env python

import rospy
import smach
import actionlib
from aaf_walking_group.msg import InterfaceAction, InterfaceGoal

class GuideInterface(smach.State):
    def __init__(self, waypointset):
        smach.State.__init__(
            self,
            outcomes=['move_to_point', 'aborted', 'killall'],
            input_keys=['current_waypoint'],
            output_keys=['waypoint']
        )
        self.waypointset = waypointset

        rospy.loginfo("Creating guide interface client...")
        self._client = actionlib.SimpleActionClient(
            'interface_server',
            InterfaceAction
        )
        self._client.wait_for_server()
        rospy.loginfo("...done")

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

        # Getting the next waypoint from guide interface
        rospy.loginfo("Opening the guide interface...")
        goal = InterfaceGoal()
        goal.possible_points = waypoints
        goal.next_point = next_waypoint
        rospy.loginfo("Sending a goal to interface server...")
        self._client.send_goal_and_wait(goal)
        result = self._client.get_result()
        rospy.loginfo("Got the chosen next waypoint.")
        next_waypoint = result.chosen_point

        rospy.loginfo("I will go to: " + next_waypoint)
        userdata.waypoint = next_waypoint
        return 'move_to_point'
