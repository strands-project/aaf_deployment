#!/usr/bin/python
import rospy
import roslib

import web
import pymongo
import os
import json
import requests
import datetime

from threading import Lock
import pygame

WEATHER_URL = "http://api.openweathermap.org/data/2.5/weather?q=Vienna,Austria"

### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('info_terminal_gui') + '/www'
render = web.template.render(TEMPLATE_DIR, base='base', globals={})
os.chdir(TEMPLATE_DIR) # so that the static content can be served from here.

class TranslatedStrings(object):
    def __init__(self, language):
        # Load translated strings
        self.language =  language
        with open(TEMPLATE_DIR + "/localisation.json") as f:
            strings =  json.load(f)
        self.translations = {}
        for s in strings:
            try:
                self.translations[s] = strings[s][language]
            except:
                self.translations[s] = s
                
    def __getitem__(self, string):
        if not self.translations.has_key(string):
            rospy.logwarn("String '%s' not translatable" % string)
            return string
        return self.translations[string]
        
strings =  TranslatedStrings(rospy.get_param("language", default="EN"))

class MusicTracks(object):
    def __init__(self):
        with open(TEMPLATE_DIR+"/music-files/index.json", "r") as f:
            self.available =  json.load(f)
        self.active =  -1
        
    def play(self, track):
        pygame.init()
        pygame.mixer.init()
        pygame.mixer.music.load("music-files/playlist/" + self.available[int(track)][1])
        pygame.mixer.music.play()
        self.active =  track
        
    def stop(self):
        pygame.mixer.music.stop()
        pygame.mixer.music.stop()
        print "STOP NOW STOP NOW"
        self.active =  -1
        
tracks =  MusicTracks()

urls = (
    '/', 'MasterPage', 
    '/menu',  'Menu', 
    '/weather', 'Weather',
    '/events', 'Events',
    '/go_away', 'GoAway',
    '/play_music', 'PlayMusic', 
    '/play_item/(.*)',  'PlayTrack'
)

app = web.application(urls, globals())

class MasterPage(object):        
    def GET(self):
        return render.index(strings, datetime)

class Menu(object):
    def GET(self):
        menu =  "Death by Schnitzel"
        return render.menu(menu)

class Weather(object):
    def GET(self):
        weather =  json.loads(requests.get(WEATHER_URL).text)
        return render.weather(weather)

class Events(object):
    def GET(self):
        events =  [("img_url", "event_text")]
        return render.events(events)

class GoAway(object):
    def GET(self):
        return "ok"
    
class PlayMusic(object):
    def GET(self):
        print tracks.available
        return render.music(tracks.available, tracks.active)

class PlayTrack(object):
    def GET(self, track):
        track = int(track)
        print track
        if track == -1:
            print "STOP STOP STOP"
            tracks.stop()
        else:
            tracks.play(track)
            print "I am playing."
            print tracks.active
        return render.music(tracks.available, tracks.active)
    
    
if __name__ == "__main__":
    print "Init ROS node."
    rospy.init_node("infoterminal_gui")
    print "Web server starting.."
    app.run()