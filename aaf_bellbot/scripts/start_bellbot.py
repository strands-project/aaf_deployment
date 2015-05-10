#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#import rosparam
import actionlib
from actionlib_msgs.msg import GoalStatus
#from aaf_bellbot.msg import EmptyAction #, StateMachineAction, StateMachineGoal
import bellbot_action_server.msg
from roslaunch_axserver.msg import launchAction, launchGoal
from strands_executive_msgs.abstract_task_server import AbstractTaskServer


class StartBellbot(AbstractTaskServer):
    TOPIC = "/bellbot_action_server/status"
    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        self.started = False

        rospy.loginfo("Creating launch client...")
        self.launch_client = actionlib.SimpleActionClient("/launchServer", launchAction)
        self.launch_client.wait_for_server()
        rospy.loginfo(" ... done")
        self.bellbot_client = None
        rospy.loginfo(" ... starting " + name)
        super(StartBellbot, self).__init__(
            name=name,
            action_type=bellbot_action_server.msg.bellbotAction,
            interruptible=False
        )
        rospy.loginfo(" ... started " + name)

    def cb(self, msg):
        self.instance_running = msg.data

    def create(self, req):
        task = super(StartBellbot, self).create(req)
        if task.start_node_id == '':
            task.start_node_id = str(self.start)
            task.end_node_id = task.start_node_id
        if task.max_duration.secs == 0.0:
            task.max_duration.secs = 3600
        if task.priority == 0:
            task.priority = 2

        return task

    def execute(self, goal):
        self.started = False
        lg = launchGoal()
        lg.pkg = "aaf_bellbot"
        lg.launch_file = "aaf_bellbot.launch.xml"

        lg.monitored_topics.append(self.TOPIC)
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
            rospy.loginfo("Creating bellbot client...")
            self.bellbot_client = actionlib.SimpleActionClient("/bellbot_action_server", bellbot_action_server.msg.bellbotAction)
            self.bellbot_client.wait_for_server()
            rospy.loginfo(" ... done")
            #sg = bellbot_action_server.msg.bellbotGoal()
            #sg

            rospy.loginfo("Starting bellbot...")
            self.bellbot_client.send_goal(goal)
            while self.bellbot_client.get_state() == GoalStatus.PENDING and not self.server.is_preempt_requested() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo(" ... started")
            #self.bellbot_client.wait_for_result() # This does not return when killed, i.e. the goal is cancelled.
            while self.bellbot_client.get_state() == GoalStatus.ACTIVE and not self.server.is_preempt_requested() and not rospy.is_shutdown():
                rospy.sleep(1.0)
            state = self.bellbot_client.get_state()

        rospy.loginfo("Bellbot finished")

        if state == GoalStatus.SUCCEEDED:
            self.launch_client.cancel_goal()
            self.server.set_succeeded()
        elif self.server.is_preempt_requested():
            self.server.set_preempted()
        else:
            self.launch_client.cancel_goal()
            self.server.set_aborted()

    def preempt_cb(self):
        rospy.loginfo("Bellbot preemption requested")
        while not self.started and not rospy.is_shutdown(): # Wait until launch file is up, otherwise it dies nastily
            rospy.sleep(0.1)
        rospy.loginfo(" ... stopping Bellbot")
        if self.bellbot_client:
            self.bellbot_client.cancel_all_goals()
            rospy.loginfo(" ... waiting for Bellbot to die")
            while not self.bellbot_client.get_state() == GoalStatus.PREEMPTED and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo(" ... died")
        rospy.loginfo(" ... stopping launch server")
        self.launch_client.cancel_goal()
        rospy.loginfo(" ... preempted")

    def feedback_cb(self, feed):
        self.started = feed.ready


if __name__ == "__main__":
    rospy.init_node("bellbot")
    s = StartBellbot(rospy.get_name())
    rospy.spin()
