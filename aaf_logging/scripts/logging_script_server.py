#! /usr/bin/env python

import rospy
import subprocess
import signal
import os

import actionlib
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
import aaf_logging.msg

class LoggingScriptServer(AbstractTaskServer):

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        #self._action_name = name
        #self._as = actionlib.SimpleActionServer(self._action_name, aaf_logging.msg.RunNodeAction, execute_cb = self.execute_cb)
        #self._as.start()
        super(LoggingScriptServer, self).__init__(
            name=name,
            action_type=aaf_logging.msg.RunNodeAction,
            interruptible=True
        )
        rospy.loginfo('Server is up')

    def execute(self, goal):
        # decide whether recording should be started or stopped
        if goal.command == "start":
            rospy.loginfo('now the logging should start')
            command = "rosrun aaf_logging run_aaf_logger.bash"
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
            rospy.loginfo(self.p.pid)

            # check if the goal is preempted
            rate = rospy.Rate(1.0)
            while not rospy.is_shutdown() and not self.server.is_preempt_requested(): # and self.p.poll() is None:
                rate.sleep()

            rospy.loginfo('Logging is preempted')
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
            #if rospy.is_shutdown():
            #    self.server.set_preempted()
            #    return

        elif goal.command == "stop":
            rospy.loginfo('now the logging should stop')
            rospy.loginfo(self.p.pid)
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
            rospy.loginfo("I'm done")

        else:
            rospy.loginfo('goal.command is not valid')

        if not self.server.is_preempt_requested():
            self.server.set_succeeded()
        else:
            self.server.set_preempted()

    def create(self, req):
        task = super(LoggingScriptServer, self).create(req)
        if task.start_node_id == "":
            task.start_node_id = "ChargingPoint"
        if task.end_node_id == "":
            task.end_node_id = task.start_node_id
        if task.max_duration.secs == 0:
            task.max_duration.secs = 60 # Default execution time: 1min
        if task.priority == 0:
            task.priority = 5 # Always start and stop loggin with high priority.
        return task

if __name__ == '__main__':
    rospy.init_node('logging_script_server')
    LoggingScriptServer(rospy.get_name())
    rospy.spin()
