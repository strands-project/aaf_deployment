#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from mongodb_store_msgs.msg import MoveEntriesGoal, MoveEntriesAction, StringList
from aaf_logging.msg import StoreAndWaitAction
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
from strands_executive_msgs import task_utils



class StoreLogsServer(AbstractTaskServer):
    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        rospy.loginfo(" ... starting " + name)
        super(StoreLogsServer, self).__init__(
            name=name,
            action_type=StoreAndWaitAction,
            interruptible=False
        )
        rospy.loginfo(" ... waiting for move_mongodb_entries server")
        self.client = SimpleActionClient("/move_mongodb_entries", MoveEntriesAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... started " + name)

    def preempt_cb(self):
        rospy.loginfo("preempting data transfer")
        self.client.cancel_all_goals()
        self.server.set_preempted()

    def execute(self, goal):
        target = goal.wait_until
        target.secs -= 10 # Making sure the wait finishes before the window ends.

        rospy.loginfo("Starting data transfer")
        self.running = False
        g = MoveEntriesGoal()
        g.database = rospy.get_param('~database', 'message_store')
        g.collections = StringList(rospy.get_param('~collections', ''))
        hours = rospy.get_param('~past_hours', 24)
        time_ago = rospy.Duration(60 * 60 * hours)
        g.move_before = time_ago

        rospy.loginfo("Starting data upload")
        self.client.send_goal(g, feedback_cb=self.feeback_cb)
        self.client.wait_for_result()
        state = self.client.get_state()
        rospy.loginfo("Data upload complete. Waiting ...")

        while not rospy.is_shutdown() and not self.server.is_preempt_requested() and rospy.get_rostime() < target:
            rospy.sleep(1.0)

        rospy.loginfo("WAKE UP!")

        if self.server.is_preempt_requested():
            self.server.set_preempted()
        elif state == GoalStatus.SUCCEEDED:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

    def feeback_cb(self, msg):
        rospy.loginfo("Upload of '" + msg.completed[-1] + "' completed")

    def create(self, req):
        task = super(StoreLogsServer, self).create(req)
        if task.start_node_id == "":
            task.start_node_id = "ChargingPoint"
        if task.end_node_id == "":
            task.end_node_id = task.start_node_id
        if task.max_duration.secs == 0:
            task.max_duration = task.end_before - task.start_after
        if task.priority == 0:
            task.priority = 5 # make sure we do this.

        task_utils.add_time_argument(task, task.end_before)
        return task

if __name__ == "__main__":
    rospy.init_node("store_logs")
    l1 = StoreLogsServer(rospy.get_name())
    rospy.spin()

