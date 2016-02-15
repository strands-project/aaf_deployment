#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store.util as dc_util
from mongodb_store.srv import *
import pymongo
import bson.json_util
import json
from std_msgs.msg import String

MongoClient = dc_util.import_MongoClient()

class MonitorLogs(object):
    def __init__(self):
        rospy.init_node("monitor_logs")

        have_dc = dc_util.wait_for_mongo()
        if not have_dc:
            raise Exception("No Datacentre?")

        self._mongo_client=pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                               rospy.get_param("mongodb_port"))

        #self.topics_to_check = rospy.get_param("topics", "/tf \
        #                                                  /scan \
        #                                                  /odom \
        #                                                  /robot_pose \
        self.topics_to_check = rospy.get_param("topics", "/amcl_pose \
                                                          /wifiscanner \
                                                          /current_node \
                                                          /current_edge \
                                                          /map \
                                                          /topological_map \
                                                          /rosout_filtered \
                                                          /diagnostics_agg \
                                                          /topological_navigation/Route \
                                                          /topological_navigation/Statistics \
                                                          /current_node \
                                                          /current_edge \
                                                          /closest_node \
                                                          /do_backtrack/goal \
                                                          /speak/goal \
                                                          /strands_emails/goal \
                                                          /strands_image_tweets/goal \
                                                          /chargingServer/goal \
                                                          /chargingServer/result \
                                                          /chargingServer/cancel \
                                                          /docking/goal \
                                                          /docking/result \
                                                          /docking/cancel \
                                                          /undocking/goal \
                                                          /undocking/result \
                                                          /undocking/cancel \
                                                          /map_updates \
                                                          /move_base/current_goal \
                                                          /charging_task/goal \
                                                          /charging_task/result \
                                                          /charging_task/cancel \
                                                          /maintenance_task/goal \
                                                          /maintenance_task/result \
                                                          /maintenance_task/cancel \
                                                          /task_executor/events \
                                                          /emergency_stop_status \
                                                          /info_terminal/active_screen \
                                                          /info_terminal/task_outcome \
                                                          /info_task_server/goal \
                                                          /info_task_server/result \
                                                          /info_task_server/cancel \
                                                          /bellbot/goal \
                                                          /bellbot/result \
                                                          /bellbot/cancel \
                                                          /walking_group_fast/goal \
                                                          /walking_group_fast/result \
                                                          /walking_group_fast/cancel \
                                                          /walking_group_slow/goal \
                                                          /walking_group_slow/result \
                                                          /walking_group_slow/cancel \
                                                          /store_logs/cancel \
                                                          /store_logs/goal \
                                                          /store_logs/result \
                                                          /strands_webserver/display_1/page \
                                                          /strands_webserver/display_2/page \
                                                          /strands_webserver/display_3/page \
                                                          /strands_webserver/display_4/page \
                                                          /strands_webserver/display_5/page \
                                                          /strands_webserver/display_6/page \
                                                          /strands_webserver/display_7/page \
                                                          /strands_webserver/display_8/page \
                                                          /strands_webserver/display_9/page")
        self.topic_list = self.topics_to_check.split()

        self.topics_map = {}

        self.pub = rospy.Publisher('monitored_logs', String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(30), self.timer_callback)

    def timer_callback(self, time):
        for topic in self.topic_list:
            #print "Getting messages for " + topic
            nbr_messages = self.find_nbr_entries("roslog", dc_util.topic_name_to_collection_name(topic))
            if topic in self.topics_map:
                old_count = self.topics_map[topic]
                self.topics_map[topic] = (nbr_messages-old_count[1], nbr_messages)
            else:
                self.topics_map[topic] = (0, nbr_messages)
            #print "Got them"
        message_string = json.dumps(self.topics_map)
        self.pub.publish(message_string)

    def find_nbr_entries(self, db, msg_collection):
        collection = self._mongo_client[db][msg_collection]
        doc = {}
        return len(dc_util.query_message_ids(collection, doc, False))

if __name__ == '__main__':
    bridge = MonitorLogs()
    rospy.spin()
