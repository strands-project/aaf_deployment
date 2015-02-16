#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import *
import actionlib
from std_msgs.msg import String

from os import listdir
from os.path import isfile, join, splitext

import pygame
import roslib
import json  

class AAFWaypointPlayer(object):
    def __init__(self):
        rospy.init_node('aaf_waypoint_player')
        self.root_path = rospy.get_param('~root_path', '')
        if len(self.root_path) == 0:
            rospy.logwarn('No root path provided!')
            return
        waypoint_sounds_str = rospy.get_param('~waypoint_sounds', '')
        if len(waypoint_sounds_str) == 0:
            rospy.logwarn('No json dictionary provided!')
            return
        self.audio_priority = 0.5
        self.waypoint_sounds = json.loads(waypoint_sounds_str)
        
        print self.waypoint_sounds
        print self.root_path

        pygame.mixer.init()
        rospy.Subscriber("/current_node", String, self.waypoint_subscriber)
        while not rospy.is_shutdown():
            # In case anything needs checking
            rospy.sleep(1)

    def waypoint_subscriber(self, waypoint):
        if waypoint.data not in self.waypoint_sounds:
            return
        pygame.mixer.music.stop()
        audio_file = join(self.root_path, self.waypoint_sounds[waypoint.data])
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()
        

if __name__ == '__main__':
    server = AAFWaypointPlayer()

