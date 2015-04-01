#!/usr/bin/env python

import rospy
import smach
from copy import deepcopy
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus
from aaf_walking_group.msg import GuidingAction, GuidingGoal
from music_player.srv import MusicPlayerService, MusicPlayerServiceRequest
from aaf_waypoint_sounds.srv import WaypointSoundsService, WaypointSoundsServiceRequest
from sound_player_server.srv import PlaySoundService


class Guiding(smach.State):
    def __init__(self, waypoints, distance):
        smach.State.__init__(
            self,
            outcomes=['reached_point', 'reached_final_point', 'key_card', 'killall'],
            input_keys=['waypoint', 'play_music'],
            output_keys=['current_waypoint', 'play_music']
        )
        self.waypoints = waypoints
        self.distance = distance
        self.last_waypoint = waypoints[str(max([int(x) for x in waypoints.keys()]))]
        self.previous_waypoint = waypoints[str(min([int(x) for x in waypoints.keys()]))]
        self.sub = None
        self.card = False
        rospy.loginfo("Creating guiding client...")
        self.nav_client = actionlib.SimpleActionClient("guiding", GuidingAction)
        self.nav_client.wait_for_server()
        rospy.loginfo(" ... done")

    def music_control(self, command):
        try:
            music_client = rospy.ServiceProxy('music_player_service', MusicPlayerService)
            rospy.loginfo("Creating waypoint sound service proxy and waiting ...")
            s = rospy.ServiceProxy('aaf_waypoint_sounds_service', WaypointSoundsService)
            if command == "play":
                music_client(MusicPlayerServiceRequest.PLAY)
                s(WaypointSoundsServiceRequest.PAUSE)
            elif command == "pause":
                music_client(MusicPlayerServiceRequest.PAUSE)
                s(WaypointSoundsServiceRequest.RESUME)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)

    def execute(self, userdata):
        rospy.loginfo("Guiding group")
        self.recall_preempt()
        try:
            s = rospy.ServiceProxy('/sound_player_service', PlaySoundService)
            s.wait_for_service()
            s("jingle_therapist_continue.mp3")
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)
        rospy.sleep(2) # Magic number :(
        self.card = False

        rospy.loginfo("I am going to: " + userdata.waypoint)
        # Action server that does all the black magic for navigation
        for elem in self.waypoints.items():
            if elem[1] == userdata.waypoint:
                key = str(int(elem[0])-1)
                if not key in self.waypoints.keys():
                    key = "1"
                self.previous_waypoint = self.waypoints[key]

        if userdata.play_music:
            self.music_control("play")
        else:
            self.music_control("pause")

        goal = GuidingGoal()
        goal.waypoint = userdata.waypoint
        goal.distance = self.distance

        self.nav_client.send_goal(goal)
        # Necessary to account for seeing the card immediately after the goal was sent.
        # That leads to the goal being pending while trying to cancel and thefore not
        # cancelling, loosing every chance of doing so. Solution: wait until not pending
        # before subscribing to card reader for preempt callback.
        while self.nav_client.get_state() == GoalStatus.PENDING:
            rospy.sleep(0.01)
        rospy.loginfo("Subscribing")
        print self.card, self.preempt_requested()
        self.sub = rospy.Subscriber("/socialCardReader/commands", String, callback=self.callback)
        self.nav_client.wait_for_result()
        state = self.nav_client.get_state()

        self.music_control("pause")
        self.sub.unregister()
        self.sub = None

        if self.preempt_requested() and not self.card:
            return 'killall'
        elif self.preempt_requested() and self.card:
            userdata.current_waypoint = self.previous_waypoint
            return 'key_card'
        else:
            try:
                s = rospy.ServiceProxy('/sound_player_service', PlaySoundService)
                s.wait_for_service()
                s("jingle_waypoint_reached.mp3")
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: %s" % e)

            if state == GoalStatus.SUCCEEDED and not goal.waypoint == self.last_waypoint:
                userdata.current_waypoint = deepcopy(userdata.waypoint)
                return 'reached_point'
            elif state == GoalStatus.SUCCEEDED and goal.waypoint == self.last_waypoint:
                userdata.current_waypoint = deepcopy(userdata.waypoint)
                return 'reached_final_point'

    def request_preempt(self):
        """Overload the preempt request method to cancel guiding."""
        self.music_control("pause")
        self.nav_client.cancel_all_goals()
        smach.State.request_preempt(self)
        rospy.logwarn("Guiding Preempted!")

    def callback(self, data):
        rospy.loginfo("got card: " + str(data.data))
        if data.data == "PAUSE_WALK":
            self.card = True
#            self.sub.unregister()
#            self.sub = None
            self.request_preempt()
