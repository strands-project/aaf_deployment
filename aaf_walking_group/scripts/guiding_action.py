#! /usr/bin/env python
import rospy
import actionlib

from aaf_walking_group.msg import GuidingAction
from aaf_walking_group.msg import EmptyAction, EmptyActionGoal
import topological_navigation.msg
from std_msgs.msg import String
from sound_player_server.srv import PlaySoundService
from move_base_msgs.msg import MoveBaseAction
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest
from dynamic_reconfigure.client import Client as DynClient
import strands_webserver.client_utils as client_utils


class GuidingServer():

    def __init__(self, name):
        rospy.loginfo("Creating "+name+" server")
        self.server = actionlib.SimpleActionServer(
            '/guiding',
            GuidingAction,
            self.execute,
            False
        )
        self.server.register_preempt_callback(self.preempt_callback)

        self.display_no = rospy.get_param("~display_no", 0)

        rospy.loginfo("Creating topo nav client...")
        self.client = actionlib.SimpleActionClient(
            '/topological_navigation',
            topological_navigation.msg.GotoNodeAction
        )
        self.client.wait_for_server()
        rospy.loginfo(" ... done ")

        rospy.loginfo("Creating wait client...")
        self.empty_client = actionlib.SimpleActionClient(
            '/wait_for_participant',
            EmptyAction
        )
        self.empty_client.wait_for_server()
        rospy.loginfo(" ... done ")

        rospy.loginfo("Creating interface dynamic reconfigure client...")
        self.dyn_client = None
        try:
            self.dyn_client = DynClient('/walking_interface_server', timeout=10.0)
            rospy.loginfo(" ... done")
        except rospy.ROSException as e:
            rospy.logwarn(e)

        rospy.loginfo("Creating move_base client...")
        self.client_move_base = actionlib.SimpleActionClient(
            '/move_base',
            MoveBaseAction
        )
        self.client_move_base.wait_for_server()
        rospy.loginfo(" ... done ")

        self.card_subscriber = rospy.Subscriber(
            "/socialCardReader/QSR_generator",
            String,
            self.card_callback,
            queue_size=1
        )

        self.node_subscriber = None

        self.pause = 0
        self.pause_points = []
        self.navgoal = topological_navigation.msg.GotoNodeGoal()
        self.current_node = None

        rospy.loginfo(" ... starting "+name)
        self.server.start()
        rospy.loginfo(" ... started "+name)

    def show_web_page(self, show):
        if self.dyn_client:
            self.dyn_client.update_configuration({"web_page": show})

    def execute(self, goal):
        self.node_subscriber = rospy.Subscriber(
            "/current_node",
            String,
            self.node_callback
        )
        try:
            nodes_service = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', GetTaggedNodes)
            nodes_service.wait_for_service()
            rospy.loginfo(" ... calling get tagged nodes service")
            req = GetTaggedNodesRequest(tag='walking_group_pause')
            res = nodes_service(req)
            self.pause_points = res.nodes
            rospy.loginfo(" ... called get tagged nodes recovery")
        except rospy.ServiceException, e:
                     rospy.logwarn("Service call failed: %s" % e)

        self.pause = 0
        self.show_web_page(True)
        self.navgoal = topological_navigation.msg.GotoNodeGoal()
        self.navgoal.target = goal.waypoint
        self.navgoal.no_orientation = goal.no_orientation
        if self.server.is_preempt_requested(): # In case the card was seen immediately this would get lost otherwise
            self.server.set_preempted()
            return
        self.client.send_goal(self.navgoal)
        while not self.current_node == goal.waypoint and not rospy.is_shutdown() and not self.server.is_preempt_requested():
            rospy.sleep(1)

        self.client.wait_for_result()
        self.show_web_page(False)
        self.node_subscriber.unregister()
        self.node_subscriber = None
        if not self.server.is_preempt_requested():
            self.server.set_succeeded()
        else:
            self.server.set_preempted()

    def node_callback(self, data):
        self.current_node = data.data
        if data.data == self.navgoal.target and self.navgoal.no_orientation: # For intermediate nodes, being in the influence are is enough
            self.client.cancel_all_goals()
#            self.client_move_base.cancel_all_goals()
        if data.data in self.pause_points and not self.pause:
            rospy.loginfo("Pausing...")
            self.show_web_page(False)
            client_utils.display_relative_page(self.display_no, 'warte.html')
            self.client.cancel_all_goals()
            self.client_move_base.cancel_all_goals()
            self.pause = 1
            try:
                s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
                s.wait_for_service()
                s("jingle_stop.mp3")
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: %s" % e)


    def preempt_callback(self):
        rospy.logwarn("Guiding action preempt requested")
        self.client.cancel_all_goals()
        self.empty_client.cancel_all_goals()


    def _on_node_shutdown(self):
        self.client.cancel_all_goals()

    def card_callback(self, data):
        if self.pause == 1 and self.server.is_active():
            # call action server
            if data.data == 'near':
                rospy.loginfo("Therapist is close enough. Show continue button")
                self.empty_client.send_goal_and_wait(EmptyActionGoal())
                try:

                    self.client.send_goal(self.navgoal)
                    self.pause = 0
                    s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
                    s.wait_for_service(timeout=0.1)
                    s("jingle_patient_continue.mp3")
                except rospy.ServiceException, e:
                    rospy.logwarn("Service call failed: %s" % e)
                rospy.loginfo("sending goal to wi")
                self.show_web_page(True)


if __name__ == '__main__':
    rospy.init_node('guiding_server')
    server = GuidingServer(rospy.get_name())
    rospy.spin()
