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
        rospy.loginfo("Creating interface client...")
        self.client_walking_interface = actionlib.SimpleActionClient(
            '/walking_interface_server',
            EmptyAction
        )
        
        
        rospy.loginfo(" ... done ")
        rospy.loginfo("Creating interface client...")
        self.client_move_base = actionlib.SimpleActionClient(
            '/move_base',
            MoveBaseAction
        )
        self.client_move_base.wait_for_server()
        
        self.client_walking_interface.wait_for_server()
        rospy.loginfo(" ... done ")
        self.card_subscriber = rospy.Subscriber(
            "/socialCardReader/QSR_generator",
            String,
            self.card_callback,
            queue_size=1
        )
        
        self.node_subscriber = rospy.Subscriber(
            "/current_node",
            String,
            self.node_callback
        )

        self.pause = 0
        self.pause_points = []
        self.navgoal = topological_navigation.msg.GotoNodeGoal()
        self.current_node = None

        rospy.loginfo(" ... starting "+name)
        self.server.start()
        rospy.loginfo(" ... started "+name)

    def execute(self, goal):
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
        self.client_walking_interface.send_goal(EmptyActionGoal())
        self.navgoal = topological_navigation.msg.GotoNodeGoal()
        self.navgoal.target = goal.waypoint
        self.client.send_goal(self.navgoal)
        while not self.current_node == goal.waypoint and not rospy.is_shutdown():
            rospy.sleep(1)
            
        self.client.wait_for_result()
        self.client_walking_interface.cancel_goal()
        if not self.server.is_preempt_requested():
            self.server.set_succeeded()
        else:
            self.server.set_preempted()
            
    def node_callback(self, data):
        if self.server.is_active():
            self.current_node = data.data
            if data.data in self.pause_points:
                rospy.loginfo("Pausing...")
                self.client.cancel_all_goals()
                self.client_move_base.cancel_all_goals()
                self.pause = 1

        
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
                self.client_walking_interface.cancel_goal()
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
                self.client_walking_interface.send_goal(EmptyActionGoal())


if __name__ == '__main__':
    rospy.init_node('guiding_server')
    server = GuidingServer(rospy.get_name())
    rospy.spin()
