#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosgraph_msgs.msg import Log

class RosoutFilter():
    def __init__(self):
        # announce
        rospy.loginfo("Starting aaf logging launcher")

        # set level below which we filter
        self.level = rospy.get_param('~level', 3)

        # register republish topic
        self.logger_pub = rospy.Publisher("/rosout_filtered", Log, queue_size=100)

        # subscribe to logging
        rospy.Subscriber("/rosout", Log, self.logger_cb)

    def logger_cb(self, logmsg):
        # if below level, don't log
        if logmsg.level < level:
            return
        # otherwise, republish into /rosout_filtered
        self.logger_pub.publish(logmsg)

if __name__ == "__main__":
    rospy.init_node("filter_rosout")
    rf  = RosoutFilter()
    rospy.spin()
