#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import rostopic
from strands_emails.msg import SendEmailAction, SendEmailGoal
from actionlib import SimpleActionClient
import time


class CheckScheduler(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.topic = rospy.get_param("~topic")
        rospy.loginfo("Waiting for message type of '%s' ..." % self.topic)
        self.ttype = rostopic.get_topic_class(self.topic, True)[0]
        rospy.loginfo("... got topic type")
        self.rate = rospy.Rate(.02)
        self.ok = True
        rospy.loginfo("Starting email client ...")
        self.client = SimpleActionClient("strands_emails", SendEmailAction)
        self.client.wait_for_server()
        rospy.loginfo("... done")
        self.message = SendEmailGoal()
        self.message.to_address = "henry.strands@hanheide.net"
        self.message.subject = "%s stopped working" % self.topic
        self.text = "The topic '"+self.topic+"' is not published any more. Last published at %s."
        rospy.loginfo("... all done")

    def spin(self):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message(self.topic, self.ttype, timeout=10.)
            except rospy.ROSException:
                if self.ok:
                    rospy.logerr("/current_schedule is not published")
                    self.message.text = self.text % str(time.asctime(time.localtime(time.time())))
#                    print self.message
                    self.client.send_goal(self.message)
                self.ok = False
            else:
                self.ok = True

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("topic_monitor")
    c = CheckScheduler(rospy.get_name())
    c.spin()
