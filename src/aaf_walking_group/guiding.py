#!/usr/bin/env python

import rospy
import smach
from copy import deepcopy
import actionlib
from aaf_walking_group.msg import GuidingAction, GuidingActionGoal, GuidingGoal
from aaf_music_player.msg import MusicPlayerService, MusicPlayerServiceRequest, MusicPlayerServiceResponse


class Guiding(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['reached_point', 'reached_final_point', 'key_card'],
            input_keys=['waypoint'],
            output_keys=['current_waypoint']
        )

    def execute(self, userdata):
        rospy.loginfo("Guiding group")
        rospy.sleep(1)
        rospy.loginfo("I am going to: " + userdata.waypoint)
        # Action server that does all the black magic for navigation
        nav_client = actionlib.SimpleActionClient("guiding", GuidingAction)
        nav_client.wait_for_server()

        try:
            music_client = rospy.ServiceProxy('aaf_music_player_service', MusicPlayerService)
            music_client(MusicPlayerServiceRequest.PLAY)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)

        goal = GuidingGoal()
        print goal
        goal.waypoint = userdata.waypoint
        nav_client.send_goal_and_wait(goal)

        try:
            music_client = rospy.ServiceProxy('aaf_music_player_service', MusicPlayerService)
            music_client(MusicPlayerServiceRequest.PAUSE)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)

        # If successful and not last point
        userdata.current_waypoint = deepcopy(userdata.waypoint)
        return 'reached_point'

        # If successful and last point
        userdata.current_waypoint = deepcopy(userdata.waypoint)
        return 'reached_final_point'

        # If aborted by showing key card
        userdata.current_waypoint = deepcopy(userdata.waypoint)
        return 'key_card'
