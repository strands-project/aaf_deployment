#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import *
import actionlib
from std_msgs.msg import String

from aaf_waypoint_sounds.srv import *

from pygame_managed_player.pygame_player import PyGamePlayer
from mongodb_media_server import MediaClient

from os import listdir
from os.path import isfile, join, splitext, exists, expanduser
from os import makedirs

import pygame
import roslib
import json

class AAFWaypointPlayer(object):
    def __init__(self):
        rospy.init_node('aaf_waypoint_player')
        self.music_set = rospy.get_param('~music_set', '')
        rospy.loginfo(self.music_set)
        if len(self.music_set) == 0:
            rospy.logwarn('No music set provided!')
            return
        self.audio_folder = join(expanduser('~'), '.ros', 'aaf_waypoint_sounds')
        if len(self.audio_folder) == 0:
            rospy.logwarn('No audio folder provided!')
            return
        waypoint_sounds_str = rospy.get_param('~waypoint_sounds', '')
        if len(waypoint_sounds_str) == 0:
            rospy.logwarn('No json dictionary provided!')
            return
        self.audio_priority = 0.5
        self.waypoint_sounds = json.loads(waypoint_sounds_str)

        hostname = rospy.get_param('mongodb_host')
        port = rospy.get_param('mongodb_port')

        self.mc = MediaClient(hostname, port)

        sets = self.mc.get_sets("Music")
        object_id = None
        for s in sets:
            if s[0] == self.music_set:
                object_id = s[2]

        if object_id is None:
            rospy.logwarn('Could not find any set in database matching %s' % self.music_set)
            return

        file_set = self.mc.get_set(object_id)

        for f in file_set:
            print "Media name:", f[0]

        file_names = [f[0] for f in file_set]

        for value in self.waypoint_sounds.itervalues():
            if not value in file_names:
                rospy.logwarn('Could not find ' + value + ' in media set')
                return

        if not exists(self.audio_folder):
            makedirs(self.audio_folder)

        for f in file_set:
            file = self.mc.get_media(str(f[2]))
            outfile = open(join(self.audio_folder, f[0]), 'wb')
            filestr = file.read()
            outfile.write(filestr)
            outfile.close()

        print "Mappings:"
        print self.waypoint_sounds
        print "Media folder:"
        print self.audio_folder

        #pygame.mixer.init()
        self.player = PyGamePlayer(0.2, 1.0, 0.5, frequency=44100)
        rospy.Subscriber("/current_node", String, self.waypoint_subscriber)

        self.enable_sounds = False
        self.service = rospy.Service('aaf_waypoint_sounds_service', WaypointSoundsService, self.toggle_service)

        while not rospy.is_shutdown():
            # In case anything needs checking
            rospy.sleep(1)

    def toggle_service(self, req):
        if req.toggle_action == WaypointSoundsServiceRequest.RESUME:
            self.enable_sounds = True
        elif req.toggle_action == WaypointSoundsServiceRequest.PAUSE:
            self.enable_sounds = False

        return WaypointSoundsServiceResponse()

    def waypoint_subscriber(self, waypoint):
        if not self.enable_sounds:
            return
        if waypoint.data not in self.waypoint_sounds:
            return
        #pygame.mixer.music.stop()
        audio_file = join(self.audio_folder, self.waypoint_sounds[waypoint.data])
        #pygame.mixer.music.load(audio_file)
        #pygame.mixer.music.play()
        self.player.play_music(audio_file, blocking=False)


if __name__ == '__main__':
    server = AAFWaypointPlayer()
