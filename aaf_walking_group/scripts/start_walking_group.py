#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
import actionlib
from actionlib_msgs.msg import GoalStatus
from aaf_walking_group.msg import EmptyAction, StateMachineAction, StateMachineGoal
from roslaunch_axserver.msg import launchAction, launchGoal
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
from std_msgs.msg import Bool
import strands_navigation_msgs.srv

class StartWalkingGroup(AbstractTaskServer):
    TOPIC = "/walking_group_action_status"
    def __init__(self, name, group, location, params, values):
        rospy.loginfo("Starting node: %s" % name)
        self.started = False
        self.group = group
        self.location = location
        self.params = params
        self.values = values

        self.instance_running = False
        self.sub = rospy.Subscriber(self.TOPIC, Bool, self.cb)
        self.pub = rospy.Publisher(self.TOPIC, Bool, queue_size=10, latch=True)
        self.pub.publish(self.instance_running)

        rospy.loginfo("Creating launch client...")
        self.launch_client = actionlib.SimpleActionClient("/launchServer", launchAction)
        self.launch_client.wait_for_server()
        rospy.loginfo(" ... done")
        self.smach_client = None
        rospy.loginfo(" ... starting " + name)
        super(StartWalkingGroup, self).__init__(
            name=name,
            action_type=EmptyAction,
            interruptible=False
        )
        rospy.loginfo(" ... started " + name)

    def cb(self, msg):
        self.instance_running = msg.data

    def create(self, req):
        task = super(StartWalkingGroup, self).create(req)
        if task.start_node_id == '':
            task.start_node_id = self.location
            task.end_node_id = task.start_node_id
        if task.max_duration.secs == 0.0:
            task.max_duration = task.end_before - task.start_after
            task.max_duration.secs -= 120. # Slack for scheduler
        if task.priority == 0:
            task.priority = 3
        return task

    def execute(self, goal):
        if self.instance_running:
            rospy.logfatal("An instance of the walking roup is already running; cannot start a second.")
            self.server.set_aborted()
            return

        self.pub.publish(True)
        self.started = False
        lg = launchGoal()
        lg.pkg = "aaf_walking_group"
        lg.launch_file = "walking_group.launch.xml"
        lg.parameters = self.params
        lg.values = self.values
        lg.monitored_topics.append("/walking_group_smach/status")
        
        topological_map_name=rospy.get_param('/walking_group_topological_map','aaf_WG_y4')
        self.switch_map(topological_map_name)

        self.launch_client.send_goal(lg, feedback_cb=self.feedback_cb)
        rospy.loginfo("Wait for launch file to start ...")
        while not self.started and not self.server.is_preempt_requested() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo(" ... started")

        if self.server.is_preempt_requested(): # Since we wait for the launch file to come up,
            state = GoalStatus.PREEMPTED       # the goal could have already been cancelled at this point
            while not self.started and not rospy.is_shutdown():
                rospy.sleep(0.1)
        else:
            rospy.loginfo("Creating walking group smach client...")
            self.smach_client = actionlib.SimpleActionClient("/walking_group_smach", StateMachineAction)
            self.smach_client.wait_for_server()
            rospy.loginfo(" ... done")
            sg = StateMachineGoal()
            sg.group = self.group
            rospy.loginfo("Starting statemachine and running %s group..." % self.group)
            self.smach_client.send_goal(sg)
            while self.smach_client.get_state() == GoalStatus.PENDING and not self.server.is_preempt_requested() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo(" ... started")
            self.smach_client.wait_for_result()
            state = self.smach_client.get_state()

        rospy.loginfo("Walking group finished")
        
        topological_map_name=rospy.get_param('/day_topological_map','aaf_y4')
        self.switch_map(topological_map_name)
        
        self.pub.publish(False)
        if state == GoalStatus.SUCCEEDED:
            self.launch_client.cancel_goal()
            self.server.set_succeeded()
        elif state == GoalStatus.PREEMPTED:
            self.server.set_preempted()
        else:
            self.launch_client.cancel_goal()
            self.server.set_aborted()


    def switch_map(self, topological_map_name):
        while not rospy.is_shutdown() and not self.server.is_preempt_requested():
            try:
                rospy.wait_for_service('/topological_map_manager/switch_topological_map', timeout=10)
                cont = rospy.ServiceProxy('/topological_map_manager/switch_topological_map', strands_navigation_msgs.srv.GetTopologicalMap)
                resp1 = cont(topological_map_name)
                if resp1.map:
                    rospy.loginfo("Switched Topological maps to: %s" %topological_map_name)
                    break
                else:
                    rospy.logwarn("Couldn't Switch Topological maps to: %s, retrying." %topological_map_name)
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed, retrying: %s"%e)
            rospy.sleep(1.)

    def preempt_cb(self):
        rospy.loginfo("Walking group preemption requested")
        while not self.started and not rospy.is_shutdown(): # Wait until launch file is up, otherwise it dies nastily
            rospy.sleep(0.1)
        rospy.loginfo(" ... stopping smach")
        if self.smach_client:
            self.smach_client.cancel_all_goals()
            rospy.loginfo(" ... waiting for smach to die")
            # Wait until state machine is dead, so everything is reset bevore killing the components.
            while not self.smach_client.get_state() == GoalStatus.PREEMPTED and not rospy.is_shutdown():
                rospy.sleep(0.1)
        rospy.loginfo(" ... stopping launch server")
        self.launch_client.cancel_goal()
        rospy.loginfo(" ... preempted")

    def feedback_cb(self, feed):
        self.started = feed.ready


class StartGroups(object):

    def __init__(self):
        rospy.loginfo("Sarting walking groups")
        paramlist=rosparam.load_file(rospy.get_param("~config_file"))[0][0]
        self.server_list = []
        for k in paramlist["walking_group"].keys():
            self.server_list.append(StartWalkingGroup(
                name=k,
                group=paramlist["walking_group"][k]["group"],
                location=paramlist["walking_group"][k]["start_location"],
                params=paramlist["walking_group"][k]["parameters"],
                values=paramlist["walking_group"][k]["values"]
            ))


if __name__ == "__main__":
    rospy.init_node("walking_group")
    s = StartGroups()
    rospy.spin()
