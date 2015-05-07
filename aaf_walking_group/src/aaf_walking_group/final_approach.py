#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus
from aaf_walking_group.msg import GuidingAction, GuidingGoal
from sound_player_server.srv import PlaySoundService


class FinalApproach(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['reached_point', 'key_card', 'killall'],
            input_keys=['waypoints', 'play_music'],
            output_keys=['waypoints', 'play_music']
        )
        self.sub = None
        self.card = False
        rospy.loginfo("Creating approach client...")
        self.nav_client = actionlib.SimpleActionClient("guiding", GuidingAction)
        self.nav_client.wait_for_server()
        rospy.loginfo(" ... done")

    def execute(self, userdata):
        rospy.loginfo("Approaching resting area")
        self.recall_preempt()
        try:
            s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
            s.wait_for_service()
            s("jingle_therapist_continue.mp3")
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)
        rospy.sleep(2) # Magic number :(
        self.card = False

        waypoint = userdata.waypoints.get_current_resting_waypoint()

        rospy.loginfo("I am going to: " + waypoint)

        goal = GuidingGoal()
        goal.waypoint = waypoint
        goal.no_orientation = False

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

        if self.preempt_requested() and not self.card:
            return 'killall'
        elif self.preempt_requested() and self.card:
            print userdata.waypoints.get_route_to_current_waypoint()
            userdata.waypoints.reverse()
            print userdata.waypoints.get_route_to_current_waypoint()
            return 'key_card'
        elif state == GoalStatus.SUCCEEDED: # and not goal.waypoint == last_waypoint:
            try:
                s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
                s.wait_for_service()
                s("jingle_waypoint_reached.mp3")
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: %s" % e)
            return 'reached_point'

    def request_preempt(self):
        """Overload the preempt request method to cancel guiding."""
        self.nav_client.cancel_all_goals()
        smach.State.request_preempt(self)
        rospy.logwarn("Final Approach Preempted!")

    def callback(self, data):
        rospy.loginfo("got card: " + str(data.data))
        if data.data == "PAUSE_WALK":
            self.card = True
            self.request_preempt()
