#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from roslaunch_axserver.msg import launchAction, launchGoal
from aaf_logging.msg import EmptyAction, EmptyGoal
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
import time
import signal as sig
import subprocess
import os


class Logger():
    def __init__(self):
        rospy.loginfo("Starting aaf logging launcher")
        self.set_running(False)
        rospy.loginfo("Creating launch client...")
        self.launch_client = actionlib.SimpleActionClient("/launchServer", launchAction)
        self.launch_client.wait_for_server()
        rospy.loginfo(" ... done")

    def set_running(self, running):
        self.running = running
        rospy.set_param("~running", self.running)

    def start_logging(self, parameters, values):
        if not self.is_running():
            rospy.loginfo("Starting logging nodes")
            self.set_running(False)
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

            command = "rosrun aaf_logging run_aaf_logger.bash"
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
            rospy.loginfo(self.p.pid)

            rospy.loginfo(" ... started")

    def stop_logging(self):
        rospy.loginfo("Logging preemption requested")
        if self.is_running():
            while not self.is_running() and not rospy.is_shutdown(): # Wait until launch file is up, otherwise it dies nastily
                rospy.sleep(0.1)
            rospy.loginfo(" ... stopping launch server")
            self.set_running(False)
            self.launch_client.cancel_goal()

            rospy.loginfo(self.p.pid)
            os.killpg(os.getpgid(self.p.pid), sig.SIGINT)
        rospy.loginfo(" ... preempted")

    def feedback_cb(self, feed):
        self.set_running(feed.ready)

    def is_running(self):
        return self.running

    def signal_handler(self, signal, frame):
        running = self.is_running()
        self.stop_logging()
        self.set_running(running)
        rospy.signal_shutdown("Shutdown requested by signal")

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
    running = rospy.get_param("~running", False)
    l  = Logger()
    l1 = LoggingServer(rospy.get_name()+"_start", l)
    l2 = LoggingServer(rospy.get_name()+"_stop", l)

    if running:
        rospy.logwarn("It appears that the logging servers did not shutdown correctly.")
        client = actionlib.SimpleActionClient(rospy.get_name()+"_start", EmptyAction)
        client.wait_for_server()
        rospy.logwarn("Restarting logging.")
        client.send_goal(EmptyGoal)

    signals = [sig.SIGINT, sig.SIGTERM] # SIGKILL cannot be caught...
    for i in signals:
        sig.signal(i, l.signal_handler)
    rospy.spin()
