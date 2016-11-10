#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from info_task.msg import EmptyAction, Clicks
from info_task.utils import Gaze, Head, PTU
from strands_executive_msgs.abstract_task_server import AbstractTaskServer
import strands_webserver.client_utils


class InfoTaskServer(AbstractTaskServer):
    def __init__(self, name, url_suffix=None):
        rospy.loginfo("Starting InfoTaskServer: %s" % name)
        # Creating action clients
        self.gaze = Gaze()
        self.head = Head()
        self.ptu = PTU()

        self.url_suffix = url_suffix

        # interaction storage
        self.interaction_times = []
        self.pages = []

        self.extension_time = rospy.get_param("~extension_time", 30.0)
        self.reset_time = 0.0

        # Creating activity subscriber and publisher
        self.pub = rospy.Publisher("/info_terminal/task_outcome", Clicks, queue_size=10, latch=True)

        rospy.loginfo(" ... starting " + name)
        super(InfoTaskServer, self).__init__(
            name=name,
            action_type=EmptyAction,
            interruptible=True
        )
        rospy.Subscriber("/info_terminal/active_screen", String, self.button_pressed_callback)
        rospy.loginfo(" ... started " + name)

    def execute(self, goal):
        self.ptu.turnPTU(-180, 10) # Looking back and slightly down
        self.head.turnHead()       # Turning head backwards
        self.gaze.people()         # Looking at upper bodies

        # Showing info terminal webserver
        if self.url_suffix is not None:
            strands_webserver.client_utils.display_url(0,
                'http://localhost:8080/?page=' + self.url_suffix)
        else:
            strands_webserver.client_utils.display_url(0,
                'http://localhost:8080/')

        rate = rospy.Rate(1)
        # preempt will not be requested while activity is happening
        while not rospy.is_shutdown() and not self.server.is_preempt_requested():
            # loop for duration
            if self.reset_time < rospy.Time.now().to_sec():
                self.interruptible = True
            rospy.logdebug("Info task active with interruptable: %s" % str(self.interruptible))
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
            task.max_duration = task.end_before - task.start_after # Default execution time: window length
        if task.priority == 0:
            task.priority = 1
        return task

    def button_pressed_callback(self, active_screen):
        # reset timeout
        if self.server.is_active():
            rospy.loginfo('button_pressed_callback')
            self.interaction_times.append(rospy.Time.now())
            self.pages.append(active_screen.data)
            self.reset_time = rospy.Time.now().to_sec() + self.extension_time
            self.interruptible = False

    def preempt_cb(self):
        clicks = Clicks()
        clicks.time_array = self.interaction_times
        clicks.page_array = self.pages

        self.interaction_times = []
        self.pages = []

        self.pub.publish(clicks)

if __name__ == "__main__":
    rospy.init_node("info_task_server")
    urls = ['news',
            'menu', 'menures',
            'info', 'weather', 'photos']
    servers = {}
    for u in urls:
        servers[u] = InfoTaskServer(
            '%s/%s' % (rospy.get_name(), u)
            )
    servers['default'] = InfoTaskServer(
        rospy.get_name()
        )

    rospy.spin()
