#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from info_task.msg import EmptyAction
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
from dynamic_reconfigure.client import Client as DynClient
from dynamic_reconfigure import DynamicReconfigureParameterException as DynException

class Server(AbstractTaskServer):
    def __init__(self, name, location, parameters):
        rospy.loginfo("Starting node: %s" % name)
        self.location = location
        self.parameters = parameters
        rospy.loginfo("Using parameters: %s" % str(self.parameters))

        rospy.loginfo(" ... starting action server " + name)
        super(Server, self).__init__(
            name=name,
            action_type=EmptyAction,
            interruptible=True
        )
        rospy.loginfo(" ... started " + name)
        rospy.loginfo(" ... done " + name)

    def create(self, req):
        task = super(Server, self).create(req)
        if task.start_node_id == '':
            task.start_node_id = self.location
            task.end_node_id = task.start_node_id
        if task.max_duration.secs == 0.0:
            task.max_duration = task.end_before - task.start_after
        if task.priority == 0:
            task.priority = 3
        return task

    def update_configureation(self, client, parameters):
        try:
            client.update_configuration(parameters)
        except DynException as e:
            rospy.logerr(e)
            rospy.logwarn("Retrying dynamic reconfigure for infremen")
            rospy.sleep(1.)
            if not rospy.is_shutdown():
                self.update_configureation(client, parameters)

    def execute(self, goal):
        try:
            rospy.loginfo("Creating infremen dynamic reconfigure client ...")
            client = DynClient("/infremen", timeout=10.)
        except rospy.ROSException as e:
            rospy.logfatal(e)
            self.server.set_aborted()
            return
        else:
            rospy.loginfo("Reconfiguring infremen ...")
            self.update_configureation(client, self.parameters)
            rospy.loginfo("... done")
            self.server.set_succeeded()
        finally:
            # If server is still running, something went wrong
            if self.server.is_active():
                self.server.set_aborted()


class StartStop(object):

    def __init__(self):
        rospy.loginfo("Sarting info terminal reconfigure tasks")
        paramlist=rosparam.load_file(rospy.get_param("~config_file"))[0][0]
        self.server_list = []
        for k, v in paramlist.items():
            self.server_list.append(Server(
                name=k,
                location=v["location"],
                parameters=v["parameters"]
            ))


if __name__ == "__main__":
    rospy.init_node("start_stop_info_terminal")
    s = StartStop()
    rospy.spin()
