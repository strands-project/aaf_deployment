#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from aaf_walking_group.msg import WalkingGroupAction, StateMachineAction, StateMachineGoal
from roslaunch_axserver.msg import launchAction, launchGoal
from strands_executive_msgs.abstract_task_server import AbstractTaskServer

class StartWalkingGroup(AbstractTaskServer):

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        self.started = False
        rospy.loginfo("Creating launch client...")
        self.launch_client = actionlib.SimpleActionClient("/launchServer", launchAction)
        self.launch_client.wait_for_server()
        rospy.loginfo(" ... done")
        self.smach_client = None
        rospy.loginfo(" ... starting " + name)
        super(StartWalkingGroup, self).__init__(
            name=name,
            action_type=WalkingGroupAction,
            interruptible=False
        )
        self.server.register_preempt_callback(self.preempt_cb)
        rospy.loginfo(" ... started " + name)

    def execute(self, goal):
        self.started = False
        lg = launchGoal()
        lg.pkg = "aaf_walking_group"
        lg.launch_file = "walking_group.launch"
        lg.parameters = goal.parameters
        lg.values = goal.values
        lg.monitored_topics.append("/walking_group_smach/status")
        self.launch_client.send_goal(lg, feedback_cb=self.feedback_cb)
        rospy.loginfo("Wait for launch file to start ...")
        while not self.started and not self.server.is_preempt_requested() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo(" ... started")

        rospy.loginfo("Creating walking group smach client...")
        self.smach_client = actionlib.SimpleActionClient("/walking_group_smach", StateMachineAction)
        self.smach_client.wait_for_server()
        rospy.loginfo(" ... done")
        sg = StateMachineGoal()
        sg.group = goal.group
        rospy.loginfo("Starting statemachine and running walking group behaviour...")
        self.smach_client.send_goal(sg)
        while self.smach_client.get_state() == GoalStatus.PENDING and not self.server.is_preempt_requested() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo(" ... started")
        self.smach_client.wait_for_result()
        state = self.smach_client.get_state()
        rospy.loginfo("Walking group finished")

        if state == GoalStatus.SUCCEEDED:
            self.server.set_succeeded()
        elif state == GoalStatus.PREEMPTED:
            self.server.set_preempted()
        else:
            self.server.set_aborted()

    def preempt_cb(self):
        while not self.started and not rospy.is_shutdown(): # Wait until launch file is up, otherwise it dies nastily
            rospy.sleep(0.1)
        self.launch_client.cancel_goal()
        if self.smach_client:
            self.smach_client.cancel_all_goals()

    def feedback_cb(self, feed):
        self.started = feed.ready

if __name__ == "__main__":
    rospy.init_node("walking_group")
    s = StartWalkingGroup(rospy.get_name())
    rospy.spin()
