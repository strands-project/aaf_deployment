#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty, EmptyResponse
from strands_executive_msgs.srv import SetExecutionStatus
from strands_executive_msgs.msg import TaskEvent
from mongodb_store.message_store import MessageStoreProxy
from scitos_msgs.srv import ResetBarrierStop, ResetMotorStop
from strands_webserver.msg import ModalDlg


class DeploymentControl(object):

    def __init__(self, name):
        # Variables
        self.maintenance_thread = None
        self.show_maintenance = False

        rospy.loginfo("Starting %s", name)
        rospy.Service('~start', Empty, self.start_cb)
        rospy.Service('~stop', Empty, self.stop_cb)
        rospy.Service('~pause', Empty, self.pause_cb)
        rospy.Service('~resume', Empty, self.resume_cb)
        rospy.Service('~reset_emergency_stop', Empty, self.reset_cb)
        rospy.Service('~show_maintenance', Empty, lambda _: self.maintenance_cb(_, True))
        rospy.Service('~hide_maintenance', Empty, lambda _: self.maintenance_cb(_, False))

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
            s = rospy.ServiceProxy("/reset_motorstop", ResetMotorStop)
            s.wait_for_service()
            rospy.loginfo(" ... calling service")
            s()
            rospy.loginfo(" ... called")

            rospy.loginfo("Creating reset_safety_stop service proxy and waiting ...")
            s = rospy.ServiceProxy("/reset_safety_stop", Empty)
            s.wait_for_service()
            rospy.loginfo(" ... calling service")
            s()
            rospy.loginfo(" ... called")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn(e)
        self.set_execution_status(True)
        return EmptyResponse()

    def walking_group_feedback(self, enable):
        services = [
            "/walking_group_recovery_feedback/disable",
            "/walking_group_recovery_feedback/enable"
        ]
        try:
            s = rospy.ServiceProxy(services[enable], Empty)
            s.wait_for_service(timeout=1)
            s()
        except (rospy.ServiceException, rospy.ROSInterruptException, rospy.ROSException) as e:
            rospy.logwarn(e)


    def maintenance_cb(self, _, show):
        self.walking_group_feedback(not show)
        rospy.sleep(1) # Not nice but have to wait until there is no one publishing any more.
        pub = rospy.Publisher("/strands_webserver/modal_dialog", ModalDlg, queue_size=1)
        m = ModalDlg()
        m.title = "Ausser Betrieb!"
        m.content = "<b>Wir arbeiten an einer L&ouml;sung. Bitte haben sie etwas Geduld.</b>"
        m.show = show
        pub.publish(m)
        return EmptyResponse()

    def _log_end(self):
        te = TaskEvent(task=None, event=TaskEvent.ROUTINE_STOPPED, time=rospy.get_rostime(), description='Routine stopped')
        self._logging_msg_store.insert(te)


if __name__ == '__main__':
    rospy.init_node('deployment_control')
    dc = DeploymentControl(rospy.get_name())
    rospy.spin()
