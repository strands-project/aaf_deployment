#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionClient
from mongodb_store_msgs.msg import MoveEntriesGoal, MoveEntriesAction, StringList
from aaf_logging.msg import EmptyAction
from strands_executive_msgs.abstract_task_server import AbstractTaskServer



class StoreLogsServer(AbstractTaskServer):
    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        rospy.loginfo(" ... starting " + name)
        super(StoreLogsServer, self).__init__(
            name=name,
            action_type=EmptyAction,
            interruptible=True
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
        rospy.loginfo("Starting data transfer")
        self.running = False
        g = MoveEntriesGoal()
        g.database = rospy.get_param('~database', 'message_store')
        g.collections = StringList(rospy.get_param('~collections', ''))
        hours = rospy.get_param('~past_hours', 24)
        time_ago = rospy.Duration(60 * 60 * hours)
        g.move_before = time_ago
        
        self.client.send_goal_and_wait(g)
        self.server.set_succeeded()

    def create(self, req):
        task = super(StoreLogsServer, self).create(req)
        if task.start_node_id == "":
            task.start_node_id = "ChargingPoint"
        if task.end_node_id == "":
            task.end_node_id = task.start_node_id
        if task.max_duration.secs == 0:
            task.max_duration.secs = 7200 # Default execution time: 2h
        if task.priority == 0:
            task.priority = 5 # make sure we do this.
        return task

if __name__ == "__main__":
    rospy.init_node("store_logs")
    l1 = StoreLogsServer(rospy.get_name())
    rospy.spin()

