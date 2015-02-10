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

WEATHER_URL = "http://api.openweathermap.org/data/2.5/weather?q=Vienna,Austria"



### Templates
template_dir = roslib.packages.get_pkg_dir('info_terminal_gui') + '/www'
render = web.template.render(template_dir, base='base', globals={})
os.chdir(template_dir) # so that the static content can be served from here.

class TranslatedStrings(object):
    def __init__(self, language):
        # Load translated strings
        self.language =  language
        with open(template_dir + "/localisation.json") as f:
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

urls = (
    '/', 'MasterPage', 
    '/menu',  'Menu', 
    '/weather', 'Weather',
    '/events', 'Events',
    '/go_away', 'GoAway',
)

app = web.application(urls, globals())

class MasterPage(object):        
    def GET(self):
        return render.index(strings, datetime)

class Menu(object):
    def GET(self):
        menu =  "Death by Scnitzel"
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

if __name__ == "__main__":
    print "Init ROS node."
    rospy.init_node("infoterminal_gui")
    print "Web server starting.."
    app.run()