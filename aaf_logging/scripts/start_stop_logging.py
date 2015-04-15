#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from roslaunch_axserver.msg import launchAction, launchGoal
from aaf_logging.msg import EmptyAction
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
import time


class Logger():
    def __init__(self):
        rospy.loginfo("Starting aaf logging launcher")
        self.running = False
        rospy.loginfo("Creating launch client...")
        self.launch_client = actionlib.SimpleActionClient("/launchServer", launchAction)
        self.launch_client.wait_for_server()
        rospy.loginfo(" ... done")

    def start_logging(self, parameters, values):
        if not self.is_running():
            rospy.loginfo("Starting logging nodes")
            self.running = False
            lg = launchGoal()
            lg.pkg = "aaf_logging"
            lg.launch_file = "loggers.launch.xml"
            lg.parameters = ['logging_tag']
            lg.parameters.extend(parameters)
            lg.values = [
                time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime(rospy.Time.now().to_sec()))
            ]
            lg.values.extend(values)
            self.launch_client.send_goal(lg, feedback_cb=self.feedback_cb)
            rospy.loginfo("Waiting...")
            while not self.is_running() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo(" ... started")

    def stop_logging(self):
        rospy.loginfo("Logging preemption requested")
        if self.is_running():
            while not self.is_running() and not rospy.is_shutdown(): # Wait until launch file is up, otherwise it dies nastily
                rospy.sleep(0.1)
            rospy.loginfo(" ... stopping launch server")
            self.running = False
            self.launch_client.cancel_goal()
        rospy.loginfo(" ... preempted")

    def feedback_cb(self, feed):
        self.running = feed.ready

    def is_running(self):
        return self.running

class LoggingServer(AbstractTaskServer):
    def __init__(self, name, llauncher):
        self.name = name
        self.llauncher = llauncher
        self.log_dir = rospy.get_param("~logging_dir", "/opt/strands/data/")
        rospy.loginfo("Starting node: %s" % name)
        rospy.loginfo(" ... starting " + name)
        super(LoggingServer, self).__init__(
            name=name,
            action_type=EmptyAction,
            interruptible=True
        )
        rospy.loginfo(" ... started " + name)

    def execute(self, goal):
        if "start" in self.name:
            self.llauncher.start_logging(
                parameters=['logging_dir'],
                values=[self.log_dir]
            )
        elif "stop" in self.name:
            self.llauncher.stop_logging()

        if not self.server.is_preempt_requested():
            self.server.set_succeeded()
        else:
            self.server.set_preempted()

    def create(self, req):
        task = super(LoggingServer, self).create(req)
        if task.start_node_id == "":
            task.start_node_id = "ChargingPoint"
        if task.end_node_id == "":
            task.end_node_id = task.start_node_id
        if task.max_duration.secs == 0:
            task.max_duration.secs = 60 # Default execution time: 1min
        if task.priority == 0:
            task.priority = 5 # Always start and stop loggin with high priority.
        return task

if __name__ == "__main__":
    rospy.init_node("logging_server")
    l  = Logger()
    l1 = LoggingServer(rospy.get_name()+"_start", l)
    l2 = LoggingServer(rospy.get_name()+"_stop", l)
    rospy.spin()

