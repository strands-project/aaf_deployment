#!/usr/bin/env python

import rospy
import smach
from copy import deepcopy
import actionlib
from aaf_walking_group.msg import GuidingAction, GuidingGoal
from music_player.srv import MusicPlayerService, MusicPlayerServiceRequest


class Guiding(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['reached_point', 'reached_final_point', 'key_card', 'killall'],
            input_keys=['waypoint'],
            output_keys=['current_waypoint']
        )
        rospy.loginfo("Creating guiding client...")
        self.nav_client = actionlib.SimpleActionClient("guiding", GuidingAction)
        self.nav_client.wait_for_server()
        rospy.loginfo(" ... done")

    def music_control(self, command):
        try:
            music_client = rospy.ServiceProxy('music_player_service', MusicPlayerService)
            if command == "play":
                music_client(MusicPlayerServiceRequest.PLAY)
            elif command == "pause":
                music_client(MusicPlayerServiceRequest.PAUSE)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)

    def execute(self, userdata):
        rospy.loginfo("Guiding group")
        rospy.sleep(1)
        rospy.loginfo("I am going to: " + userdata.waypoint)
        # Action server that does all the black magic for navigation

        self.music_control("play")

        goal = GuidingGoal()
        print goal
        goal.waypoint = userdata.waypoint
        self.nav_client.send_goal_and_wait(goal)

        self.music_control("pause")

        if self.preempt_requested():
            return 'killall'
        else:

            # If successful and not last point
            userdata.current_waypoint = deepcopy(userdata.waypoint)
            return 'reached_point'

            # If successful and last point
            userdata.current_waypoint = deepcopy(userdata.waypoint)
            return 'reached_final_point'

            # If aborted by showing key card
            userdata.current_waypoint = deepcopy(userdata.waypoint)
            return 'key_card'

    def request_preempt(self):
        """Overload the preempt request method to cancel guiding."""
        self.music_control("pause")
        self.nav_client.cancel_all_goals()
        smach.State.request_preempt(self)
        rospy.logwarn("Guiding Preempted!")
