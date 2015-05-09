#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 12:08:14 2015

@author: cdondrup
"""

import rospy
import rosparam
import rostopic
import roslib
from strands_executive_msgs.msg import TaskEvent
from actionlib import SimpleActionClient
from mary_tts.msg import maryttsAction, maryttsGoal
import itertools


class Manager(object):
    ALL = "ALL"

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        config_file = rospy.get_param("~config_file", '')
        if config_file == '':
            rospy.logfatal("No config file specified")
            raise rospy.ROSException("No config file specified")
            return
        self.config = rosparam.load_file(config_file)[0][0]
        if self.ALL in self.config.keys():
            self.all = self.config[self.ALL]["events"]
        else:
            self.all = []
        print self.config

        rospy.loginfo("Creating mary client")
        self.mary_client = SimpleActionClient("speak", maryttsAction)
        self.mary_client.wait_for_server()
        rospy.loginfo(" ... done")
        rospy.Subscriber("/task_executor/events", TaskEvent, callback=self.callback)

    def callback(self, msg):
        if msg.task.action[1:] in self.config.keys():
            events = itertools.chain(self.all, self.config[msg.task.action[1:]]["events"])
        else:
            events = self.all

        for event in events:
            execute = True
            if msg.event == eval("TaskEvent." + event.keys()[0]):
                if "compare" in event.values()[0].keys():
                    comps = event.values()[0]["compare"]
                    for comp in comps:
                        if not eval(str(comp["static_value"]) + comp["comparison"] + "msg." + comp["task_field"]):
                            execute = False
                            break
                if "compare_to_topic" in event.values()[0].keys():
                    comps = event.values()[0]["compare_to_topic"]
                    for comp in comps:
                        res = rospy.wait_for_message(comp["topic"], roslib.message.get_message_class(rostopic.get_topic_type(comp["topic"], True)[0]))
                        if not eval("res." + comp["field"] + comp["comparison"] + "msg." + comp["task_field"]):
                            execute = False
                            break
                if execute:
                    self.mary_client.send_goal_and_wait(maryttsGoal(text=event.values()[0]["text"]))
                    rospy.loginfo("Saying: " + event.values()[0]["text"])


if __name__ == "__main__":
    rospy.init_node("task_monitor")
    lm = Manager(rospy.get_name())
    rospy.spin()