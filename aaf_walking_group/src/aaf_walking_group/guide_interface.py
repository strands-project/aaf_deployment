#!/usr/bin/env python

import rospy
import smach
import actionlib
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from aaf_walking_group.msg import InterfaceAction, InterfaceGoal
from std_srvs.srv import Empty, EmptyResponse

class GuideInterface(smach.State):
    def __init__(self, waypointset):
        smach.State.__init__(
            self,
            outcomes=['move_to_point', 'aborted', 'killall'],
            input_keys=['current_waypoint', 'play_music'],
            output_keys=['waypoint', 'play_music']
        )
        self.waypointset = waypointset

        rospy.loginfo("Creating guide interface client...")
        self._client = actionlib.SimpleActionClient(
            'interface_server',
            InterfaceAction
        )
        self._client.wait_for_server()
        rospy.loginfo(" ...done")
        self.srv = None
        self.play_pub = rospy.Publisher("~play_music", Bool, queue_size=1, latch=True)
        self.srv_music = rospy.Service('/walking_group/guide_interface/toggle_music', Empty, self.toggle_play_music)

    def execute(self, userdata):
        self.srv = rospy.Service('/walking_group/guide_interface/cancel', Empty, self.cancel_srv)
        rospy.loginfo("Showing guide interface")
        rospy.sleep(1)
        self.play_music = userdata.play_music
        self.play_pub.publish(self.play_music)

        # Guide interface returning the next waypoint
        waypoints = self.waypointset
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
        goal.possible_points = waypoints.values()
        goal.next_point = next_waypoint
        rospy.loginfo("Sending a goal to interface server...")
        self._client.send_goal_and_wait(goal)
        state = self._client.get_state()
        self.srv.shutdown()
        userdata.play_music = self.play_music
        if state == GoalStatus.SUCCEEDED:
            result = self._client.get_result()
            rospy.loginfo("Got the chosen next waypoint.")
            next_waypoint = result.chosen_point

            rospy.loginfo("I will go to: " + next_waypoint)
            userdata.waypoint = next_waypoint
            return 'move_to_point'
        elif self._preempt_requested:
            return 'killall'
        else:
            return 'aborted'

    def request_preempt(self):
        """Overload the preempt request method to cancel interface goal."""
        self._client.cancel_all_goals()
        smach.State.request_preempt(self)
        self.srv.shutdown()
        rospy.logwarn("Guide Interface Preempted!")

    def cancel_srv(self, req):
        rospy.loginfo("Cancelation of guide inteface requested.")
        self._client.cancel_all_goals()
        return EmptyResponse()

    def toggle_play_music(self, req):
        self.play_music = not self.play_music
        self.play_pub.publish(self.play_music)
        return EmptyResponse()
