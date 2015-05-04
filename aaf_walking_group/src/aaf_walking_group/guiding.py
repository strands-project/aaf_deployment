#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus
from aaf_walking_group.msg import GuidingAction, GuidingGoal
from music_player.srv import MusicPlayerService, MusicPlayerServiceRequest
from aaf_waypoint_sounds.srv import WaypointSoundsService, WaypointSoundsServiceRequest
from sound_player_server.srv import PlaySoundService


class Guiding(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['reached_point', 'reached_final_point', 'key_card', 'killall'],
            input_keys=['waypoints', 'play_music'],
            output_keys=['waypoints', 'play_music']
        )
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
        last_waypoint = userdata.waypoints.get_resting_waypoints()[-1]
        self.recall_preempt()
        try:
            s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
            s.wait_for_service()
            s("jingle_therapist_continue.mp3")
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)
        rospy.sleep(2) # Magic number :(
        self.card = False

        if userdata.play_music:
            self.music_control("play")
        else:
            self.music_control("pause")

        print userdata.waypoints.get_route_to_current_waypoint()
        next_waypoint = userdata.waypoints.get_current_waypoint_in_route()

        while not rospy.is_shutdown() and not self.preempt_requested():
            rospy.loginfo("I am going to: " + next_waypoint)

            goal = GuidingGoal()
            goal.waypoint = next_waypoint
            goal.no_orientation = True

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

            self.sub.unregister()
            self.sub = None

            try:
                if not self.preempt_requested():
                    next_waypoint = userdata.waypoints.advance_on_route()
            except IndexError:
                userdata.waypoints.reverse_on_route() # Necessary to resume after interupting final_approach
                break

        self.music_control("pause")

        if self.preempt_requested() and not self.card:
            return 'killall'
        elif self.preempt_requested() and self.card:
            userdata.waypoints.reverse()
            return 'key_card'
        else:
            if state == GoalStatus.SUCCEEDED and not goal.waypoint == last_waypoint:
                try:
                    s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
                    s.wait_for_service()
                    s("jingle_waypoint_reached.mp3")
                except rospy.ServiceException, e:
                    rospy.logwarn("Service call failed: %s" % e)
                return 'reached_point'
            elif state == GoalStatus.SUCCEEDED and goal.waypoint == last_waypoint:
                try:
                    s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
                    s.wait_for_service()
                    s("jingle_waypoint_reached.mp3")
                except rospy.ServiceException, e:
                    rospy.logwarn("Service call failed: %s" % e)
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
            self.request_preempt()
