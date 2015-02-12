#!/usr/bin/python
import rospy
import roslib

import web
import pymongo
from bson import json_util, Binary
import os
import json
import requests
import datetime
from std_msgs.msg import Int8
from threading import Lock
import pygame
import xmltodict

#WEATHER_URL = "http://api.openweathermap.org/data/2.5/weather?q=Vienna,Austria"
WEATHER_URL = "http://api.worldweatheronline.com/free/v2/weather.ashx?key=c2db94527204450837f1cf7b7772b&q=Vienna,Austria&num_of_days=2&tp=3&format=json"

NEWS_URL = "http://feeds.bbci.co.uk/news/world/rss.xml"

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
        self.active =  -1
        
tracks =  MusicTracks()

urls = (
    '/', 'MasterPage', 
    '/menu',  'Menu', 
    '/weather', 'Weather',
    '/events', 'Events',
    '/go_away', 'GoAway',
    '/play_music', 'PlayMusic', 
    '/play_item/(.*)',  'PlayTrack', 
    '/photo_album/(.*)',  'PhotoAlbum', 
    '/photo_album_manager', 'PhotoAlbumManager', 
    '/photo_album_manager/delete/(.*)', 'DeleteAlbumImage',
    '/get_photo/(.*)', 'GetPhoto'
)

    
# A ROS publisher for click-feedback
active_screen_pub =  rospy.Publisher("/info_terminal/active_screen", Int8, queue_size=1)
    

app = web.application(urls, globals())

def mongo_client():
    mongo = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                            rospy.get_param("mongodb_port"))
    return mongo.info_terminal
def get_image_count():
    return mongo_client().photos.find().count()


class MasterPage(object):        
    def GET(self):
        active_screen_pub.publish(MasterPage.id)
        return render.index(strings, datetime)

class Menu(object):
    def GET(self):
        active_screen_pub.publish(Menu.id)
        menu =  "Death by Schnitzel"
        return render.menu(menu)

class Weather(object):
    def GET(self):
        active_screen_pub.publish(Weather.id)
        try:
            weather =  json.loads(requests.get(WEATHER_URL).text)
        except:
            return render.index(strings, datetime)
        return render.weather(weather)

class Events(object):
    def GET(self):
        active_screen_pub.publish(Events.id)
        news =  xmltodict.parse(requests.get(NEWS_URL).text)
        events =  []
        for n in news['rss']['channel']['item']:
            events.append((n["media:thumbnail"][0]["@url"],
                           "<h3>"+n["title"]+"</h3><h4>"+n["description"] +"</h4>"))
        return render.events(events[:3])

class GoAway(object):
    def GET(self):
        active_screen_pub.publish(GoAway.id)
        return "ok"
    
class PlayMusic(object):
    def GET(self):
        active_screen_pub.publish(PlayMusic.id)
        return render.music(tracks.available, tracks.active)

class PlayTrack(object):
    def GET(self, track):
        active_screen_pub.publish(PlayTrack.id)
        track = int(track)
        if track == -1:
            tracks.stop()
        else:
            tracks.play(track)
        return render.music(tracks.available, tracks.active)
    
class PhotoAlbum(object):
    def GET(self, image_id):
        active_screen_pub.publish(PhotoAlbum.id)
        image_id = int(image_id)
        current_image =  image_id
        count =  get_image_count()
        next_image =  image_id + 1
        if next_image == count:
            next_image = 0
        prev_image =  image_id - 1
        if prev_image < 0:
            prev_image = count - 1
        return render.photos(current_image, next_image, prev_image)
    
class PhotoAlbumManager(object):
    def GET(self):
        image_count = get_image_count()
        return render.album_manager(image_count)
    def POST(self):
        x = web.input(myfile={})
        mongo_client().photos.insert({"name": x['myfile'].filename,
                                      "content": Binary(x['myfile'].value),})
        raise web.seeother('/photo_album_manager')

class DeleteAlbumImage(object):
    def GET(self, image_id):
        # This inefficiency of my lazy mongo connection is pure shocking.
        for i, image in enumerate(mongo_client().photos.find()):
            if i == int(image_id):
                mongo_client().photos.remove(image)
                break
        raise web.seeother('/photo_album_manager')

class GetPhoto(object):
    def GET(self, image_id):
        # web.py serve an image
        for i, image in enumerate(mongo_client().photos.find()):
            if i == int(image_id):
                print "Ok I have it"
                break
        else:
            raise web.notfound()
        name =  image["name"]
        ext = name.split(".")[-1].lower() # Gather extension

        cType = {
            "png":"images/png",
            "jpg":"images/jpeg",
            "gif":"images/gif",
            "ico":"images/x-icon"            }
        
        web.header("Content-Type", cType[ext]) # Set the Header
        return image['content']
        
    
# Give each URL a unique number so that we can feedback which screen is active
# as a ROS message
for i, u in enumerate(urls):
    if u.startswith("/"):
        continue
    globals()[u].id = i
    
    
if __name__ == "__main__":
    print "Init ROS node."
    rospy.init_node("infoterminal_gui")
    print "Web server starting.."
    app.run()