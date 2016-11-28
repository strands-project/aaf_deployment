#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray
from strands_webserver.msg import ModalDlg
from std_srvs.srv import Empty, EmptyResponse


class Dialogue():
    def __init__(self):
        self.__visible = False
        self.enabled = True
        self.pub = rospy.Publisher("/strands_webserver/modal_dialog", ModalDlg, queue_size=1)

        self.m = ModalDlg()
        self.m.title = "Ich stecke fest!"
        self.m.content = "<b>Ich versuche gerade mich zu befreien. Bitte haben Sie etwas Geduld.</b>"

    def __call__(self, show):
        if self.enabled:
            if show is not self.__visible:
                self.m.show = show
                self.pub.publish(self.m)
                self.__visible = show

    def enable(self, _, return_value=None):
        self.enabled = True
        return return_value

    def disable(self, _, return_value=None):
        self(False)
        self.enabled = False
        return return_value


class RecoveryFeedback(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.path_stamp = rospy.Time.now().secs
        self.dialogue = Dialogue()
        rospy.on_shutdown(lambda: self.dialogue(False))
        rospy.Service("~enable", Empty, lambda x: self.dialogue.enable(x, EmptyResponse()))
        rospy.Service("~disable", Empty, lambda x: self.dialogue.disable(x, EmptyResponse()))
        rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self.path_cb)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.state_cb)
        rospy.loginfo("... done")

    def path_cb(self, msg):
        self.path_stamp = msg.header.stamp.secs

    def state_cb(self, msg):
        if msg.status_list:
            if msg.status_list[-1].status == 1:
                dif = abs(msg.header.stamp.secs - self.path_stamp)
                if dif > 5.:
                    rospy.logwarn("Robot appears to be stuck.")
                    self.dialogue(True)
                else:
                    self.dialogue(False)
            else:
                self.dialogue(False)


if __name__ == "__main__":
    rospy.init_node("walking_group_recovery_feedback")
    RecoveryFeedback(rospy.get_name())
    rospy.spin()

