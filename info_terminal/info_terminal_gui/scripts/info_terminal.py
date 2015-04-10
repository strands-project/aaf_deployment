#!/usr/bin/python
import rospy
import roslib

import web
import os
import json
import requests
import datetime
from std_msgs.msg import Int32
import xmltodict
import signal

from mongodb_media_server import MediaClient

#WEATHER_URL = "http://api.openweathermap.org/data/2.5/weather?q=Vienna,Austria"
WEATHER_URL = "http://api.worldweatheronline.com/free/v2/weather.ashx?key=c2db94527204450837f1cf7b7772b&q=Vienna,Austria&num_of_days=2&tp=3&format=json"
BBC_NEWS_URL = "http://feeds.bbci.co.uk/news/world/rss.xml"
HENRY_BLOG_URL = "https://henrystrands.wordpress.com/feed/"
NEWS_URL = "http://rss.orf.at/wien.xml"

### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('info_terminal_gui') + '/www'
render = web.template.render(TEMPLATE_DIR, base='base', globals={})
os.chdir(TEMPLATE_DIR) # so that the static content can be served from here.


class TranslatedStrings(object):
    def __init__(self, language):
        # Load translated strings
        self.language = language
        with open(TEMPLATE_DIR + "/localisation.json") as f:
            strings = json.load(f)
        self.translations = {}
        for s in strings:
            try:
                self.translations[s] = strings[s][language]
            except:
                self.translations[s] = s

    def __getitem__(self, string):
        if not string in self.translations:
            rospy.logwarn("String '%s' not translatable" % string)
            return string
        return self.translations[string]


class InfoTerminalGUI(web.application):
    def __init__(self):
        self.urls = (
            '/', 'MasterPage',
            '/menu',  'Menu',
            '/weather', 'Weather',
            '/events', 'Events',
            '/go_away', 'GoAway',
            '/photo_album/(.*)',  'PhotoAlbum',
        )
        web.application.__init__(self, self.urls, globals())
        signal.signal(signal.SIGINT, self._signal_handler)

        # A ROS publisher for click-feedback
        self._active_screen_pub = rospy.Publisher("/info_terminal/active_screen",
                                                  Int32, queue_size=1)
        self.string = None
        self.port = 8080

    def run(self, port, language, *middleware):
        self.strings = TranslatedStrings(language)
        self.port = port

        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', self.port))

    def _signal_handler(self, signum, frame):
        self.stop()
        print "InfoTerminal GUI stopped."

    def publish_feedback(self, page_id):
        self._active_screen_pub.publish(page_id)

app =  InfoTerminalGUI()


class MasterPage(object):
    def GET(self):
        app.publish_feedback(MasterPage.id)
        return render.index(app.strings, datetime)


class Menu(object):
    def GET(self):
        app.publish_feedback(Menu.id)
        return render.menu({})


class Weather(object):
    def GET(self):
        app.publish_feedback(Weather.id)
        try:
            weather = json.loads(requests.get(WEATHER_URL).text)
        except:
            return render.index(app.strings, datetime)
        return render.weather(weather)


class Events(object):
    def GET(self):
        app.publish_feedback(Events.id)
        blog = xmltodict.parse(requests.get(HENRY_BLOG_URL).text)
        blog_events = []
	items = blog['rss']['channel']['item']
        if not type(items) is list:
            items = [items]		
        for n in items:
            blog_events.append("<h3>"+n["title"]+"</h3><h4>"+n["description"] +"</h4>")

        news = xmltodict.parse(requests.get(NEWS_URL).text)
        events = []
        items = news['rdf:RDF']['item']
        if not type(items) is list:
            items = [items]
        for n in items:
            events.append((None,
                           "<h3>"+n["title"]+"</h3><h4>"+n["description"] +"</h4>"))
        return render.events(blog_events[:3], events[:3])


class GoAway(object):
    def GET(self):
        app.publish_feedback(GoAway.id)
        return "ok"


class PhotoAlbum(object):
    photos = None

    def GET(self, image_id):
        if PhotoAlbum.photos is None or image_id == "":
            # Rescan the info-terminal photo album in the media server.
            print "Rescanning info-terminal photo set."
            mc = MediaClient(rospy.get_param('mongodb_host'),
                             rospy.get_param('mongodb_port'))
            PhotoAlbum.photos = mc.get_set(set_type_name="Photo/info-terminal")
        app.publish_feedback(PhotoAlbum.id)
        if image_id == "":
            image_id = 0
        image_id = int(image_id)
        current_image = image_id
        count = len(PhotoAlbum.photos)
        next_image = image_id + 1
        if next_image == count:
            next_image = 0
        prev_image = image_id - 1
        if prev_image < 0:
            prev_image = count - 1
        return render.photos(PhotoAlbum.photos[current_image][0],
                             next_image, prev_image)

# Give each URL a unique number so that we can feedback which screen is active
# as a ROS message
for i, u in enumerate(app.urls):
    if u.startswith("/"):
        continue
    globals()[u].id = i

if __name__ == "__main__":
    print "Init ROS node."
    rospy.init_node("infoterminal_gui")
    print "InfoTerminal GUI Web server starting.."
    port = rospy.get_param("~port", 8080)
    language = rospy.get_param("~language", "EN")
    app.run(port, language)
