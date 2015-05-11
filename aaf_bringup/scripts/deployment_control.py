#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty, EmptyResponse
from strands_executive_msgs.srv import SetExecutionStatus
from strands_executive_msgs.msg import TaskEvent
from mongodb_store.message_store import MessageStoreProxy
from scitos_msgs.srv import ResetBarrierStop, ResetMotorStop


class DeploymentControl(object):

    def __init__(self, name):
        # Variables
        rospy.loginfo("Starting %s", name)
        rospy.Service('~start', Empty, self.start_cb)
        rospy.Service('~stop', Empty, self.stop_cb)
        rospy.Service('~pause', Empty, self.pause_cb)
        rospy.Service('~resume', Empty, self.resume_cb)
        rospy.Service('~reset_emergency_stop', Empty, self.reset_cb)

        self._logging_msg_store = MessageStoreProxy(collection='task_events')
        rospy.on_shutdown(self._log_end)

        rospy.loginfo("%s: ... started.", name)

    def set_execution_status(self, status):
        try:
            rospy.loginfo("Creating task_executor service proxy and waiting ...")
            s = rospy.ServiceProxy("/task_executor/set_execution_status", SetExecutionStatus)
            s.wait_for_service()
            rospy.loginfo(" ... calling service")
            s(status)
            rospy.loginfo(" ... called")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn(e)

    def start_cb(self, req):
        self.set_execution_status(True)
        te = TaskEvent(task=None, event=TaskEvent.ROUTINE_STARTED, time=rospy.get_rostime(), description='Routine started')
        self._logging_msg_store.insert(te)
        return EmptyResponse()

    def stop_cb(self, req):
        self.set_execution_status(False)
        self._log_end()
        return EmptyResponse()

    def pause_cb(self, req):
        self.set_execution_status(False)
        return EmptyResponse()

    def resume_cb(self, req):
        self.set_execution_status(True)
        return EmptyResponse()

    def reset_cb(self, req):
        try:
            rospy.loginfo("Creating reset_barrier_stop service proxy and waiting ...")
            s = rospy.ServiceProxy("/reset_barrier_stop", ResetBarrierStop)
            s.wait_for_service()
            rospy.loginfo(" ... calling service")
            s()
            rospy.loginfo(" ... called")

            rospy.loginfo("Creating reset_motorstop service proxy and waiting ...")
            s = rospy.ServiceProxy("/reset_barrier_stop", ResetMotorStop)
            s.wait_for_service()
            rospy.loginfo(" ... calling service")
            s()
            rospy.loginfo(" ... called")

            rospy.loginfo("Creating reset_safety_stop service proxy and waiting ...")
            s = rospy.ServiceProxy("/reset_barrier_stop", Empty)
            s.wait_for_service()
            rospy.loginfo(" ... calling service")
            s()
            rospy.loginfo(" ... called")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn(e)
        return EmptyResponse()

    def _log_end(self):
        te = TaskEvent(task=None, event=TaskEvent.ROUTINE_STOPPED, time=rospy.get_rostime(), description='Routine stopped')
        self._logging_msg_store.insert(te)


if __name__ == '__main__':
    rospy.init_node('deployment_control')
    dc = DeploymentControl(rospy.get_name())
    rospy.spin()
