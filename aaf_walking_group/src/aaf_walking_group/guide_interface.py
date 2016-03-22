#!/usr/bin/env python

import rospy
import smach
import actionlib
from std_msgs.msg import Bool, String
from actionlib_msgs.msg import GoalStatus
from aaf_walking_group.msg import InterfaceAction, InterfaceGoal
from std_srvs.srv import Empty, EmptyResponse
import aaf_walking_group.utils as utils
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest

class GuideInterface(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['move_to_point', 'aborted', 'killall'],
            input_keys=['waypoints', 'play_music'],
            output_keys=['waypoints', 'play_music']
        )
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
        self.srv_quieter = rospy.Service('/walking_group/guide_interface/volume_quieter', Empty, self.volume_control_quieter)
        self.srv_louder = rospy.Service('/walking_group/guide_interface/volume_louder', Empty, self.volume_control_louder)

    def get_tagged_nodes(self, tag):
        try:
            s = rospy.ServiceProxy("/topological_map_manager/get_tagged_nodes", GetTaggedNodes)
            s.wait_for_service()
            return s(GetTaggedNodesRequest(tag=tag)).nodes
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logerr(e)

    def execute(self, userdata):
        self.srv = rospy.Service('/walking_group/guide_interface/cancel', Empty, self.cancel_srv)
        rospy.loginfo("Showing guide interface")
        rospy.sleep(1)
        self.play_music = userdata.play_music
        self.play_pub.publish(self.play_music)

        # Guide interface returning the next waypoint
        rospy.loginfo("Current waypoint: " + userdata.waypoints.get_current_resting_waypoint())
        if rospy.wait_for_message("/closest_node", String).data in self.get_tagged_nodes(userdata.waypoints.get_current_resting_tag()):
            next_waypoint = userdata.waypoints.advance()[userdata.waypoints.RESTING]
        else:
            next_waypoint = userdata.waypoints.get_current_waypoint()[userdata.waypoints.RESTING]
        rospy.loginfo("Next waypoint in list: " + next_waypoint)

        # Getting the next waypoint from guide interface
        rospy.loginfo("Opening the guide interface...")
        goal = InterfaceGoal()
        goal.possible_points = userdata.waypoints.get_resting_waypoints()
        goal.next_point = next_waypoint
        goal.idx = userdata.waypoints.get_index()
        rospy.loginfo("Sending a goal to interface server...")
        self._client.send_goal_and_wait(goal)
        state = self._client.get_state()
        self.srv.shutdown()
        userdata.play_music = self.play_music
        if state == GoalStatus.SUCCEEDED:
            result = self._client.get_result()
            rospy.loginfo("Got the chosen next waypoint.")
            userdata.waypoints.set_index(result.idx)
            userdata.waypoints.create_route()
            rospy.loginfo("Following route: %s" % str(userdata.waypoints.get_route_to_current_waypoint()))
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

    def volume_control_quieter(self, req):
        utils.set_master_volume(utils.get_master_volume() - 5)
        return EmptyResponse()

    def volume_control_louder(self, req):
        utils.set_master_volume(utils.get_master_volume() + 5)
        return EmptyResponse()
