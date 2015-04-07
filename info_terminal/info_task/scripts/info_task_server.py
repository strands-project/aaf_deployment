#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from info_task.msg import EmptyAction, Clicks
from info_task.utils import Gaze, Head, PTU
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
import strands_webserver.client_utils


class InfoTaskServer(AbstractTaskServer):
    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        # Creating action clients
        self.gaze = Gaze()
        self.head = Head()
        self.ptu = PTU()

        # interaction storage
        self.interaction_times = []
        self.pages = []

        # Creating activity subscriber and publisher
        rospy.Subscriber("info_terminal/active_screen", Int32, self.button_pressed_callback)
        self.pub = rospy.Publisher("/info_terminal/task_outcome", Clicks, queue_size=10, latch=True)

        rospy.loginfo(" ... starting " + name)
        super(InfoTaskServer, self).__init__(
            name=name,
            action_type=EmptyAction,
            interruptible=True
        )
        rospy.loginfo(" ... started " + name)

    def execute(self, goal):
        self.ptu.turnPTU(-180, 10) # Looking back and slightly down
        self.head.turnHead()       # Turning head backwards
        self.gaze.people()         # Looking at upper bodies

        # Showing info terminal webserver
        strands_webserver.client_utils.display_url(0, 'http://localhost:8080')

        rate = rospy.Rate(1)
        # preempt will not be requested while activity is happening
        while not rospy.is_shutdown() and not self.server.is_preempt_requested():
            # loop for duration
            rate.sleep()

        # Reset ptu, head and gaze
        self.ptu.turnPTU(0, 0)
        self.gaze.preempt()
        self.head.resetHead()

        if not self.server.is_preempt_requested():
            self.server.set_succeeded()
        else:
            self.server.set_preempted()

    def create(self, req):
        task = super(InfoTaskServer, self).create(req)
        if task.max_duration.secs == 0:
            task.max_duration.secs = 600 # Default execution time: 10min
        return task

    def button_pressed_callback(self, active_screen):
        # reset timeout
        rospy.loginfo('button_pressed_callback')
        self.interaction_times.append(rospy.Time.now())
        self.pages.append(active_screen.data)

    def preempt_cb(self):
        clicks = Clicks()
        clicks.time_array = self.interaction_times
        clicks.page_array = self.pages

        self.interaction_times = []
        self.pages = []

        self.pub.publish(clicks)

if __name__ == "__main__":
    rospy.init_node("info_task_server")
    i = InfoTaskServer(rospy.get_name())
    rospy.spin()

