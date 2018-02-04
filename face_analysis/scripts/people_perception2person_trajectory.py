#!/usr/bin/python
'''
----------------------------------
USER ANALYSIS
----------------------------------
'''


import math
import rospy
import os
import argparse


from std_msgs.msg import String
from trajectory_analysis_gui.srv import *
from ms_face_api.srv import *
from ms_face_api.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import opencv_tools


from threading import Thread
import json
import pymongo 
import datetime
import time

import collections
import glob



import wx
from wx.lib import statbmp

import cv, cv2
import numpy as np
import traceback
global mongo


class MongoDb():
    

        
    def __init__(self,path_videos,video_folders,advancedfilter) :
        
        
        self.cvframe=None
        self.index_queries=-1
        self.length_queries=0
        try:
            self.client = pymongo.MongoClient('localhost', 62345)
            
            #self.client = pymongo.MongoClient('10.5.42.18', 23456)
            self.db_message_store = self.client['message_store_y4']
            self.db_analysis = self.client['final_users_analysis']  
            self.collection_people_perception = self.db_message_store['people_perception']
            self.collection_trajectory_person= self.db_analysis['trajectory_person']
            self.collection_trajectory_person_filtered= self.db_analysis['trajectory_person_filtered'] 

            self.msgstore=True
        except:
            self.msgstore=False
            print ("Message Store not available")   


        self.videofolder=None
        self.videopath=path_videos
        self.arrayvideosfolder=video_folders
        
        
        self.min_time_detected=2.0 ##minimum secs between first and last detection to consider as a person  
        self.max_time_detected=600.0 ##maximum secs between first and last detection to consider as a person  


        if advancedfilter is True:
            
            self.advanced_filter()
            
        else:
            self.people_perception_2_person_trajectory()

        
    def people_perception_2_person_trajectory(self):
        
        for self.videofolder in self.arrayvideosfolder:
            

            with open(self.videopath+'/info_videos_'+self.videofolder+'.json', 'r') as f:
                videodata = json.load(f)
                
            for ivideo in videodata['video']:
    
                if ivideo['name']!='':
                    
                    name_file=ivideo['name']
                                   
                    print 'name_file',name_file
                    self.cvframe=opencv_tools.ExtractFrame(self.videopath,self.videofolder,name_file)                    
                    print 'ExtractFrame'
                    
                    start_timestamp=int(self.cvframe.array_timestamps[0]['timestamp'])
                    end_timestamp=int(self.cvframe.array_timestamps[len(self.cvframe.array_timestamps)-1]['timestamp'])
                    #start_timestamp=1483282169 
                    
                    query_start_timestamp=datetime.datetime.fromtimestamp(start_timestamp)
                    query_start_timestamp=query_start_timestamp+ datetime.timedelta(hours=1)

                    query_end_timestamp=datetime.datetime.fromtimestamp(end_timestamp)
                    query_end_timestamp=query_end_timestamp+ datetime.timedelta(hours=1)    
                                   

                    query = {"$and": [{'_meta.inserted_at': {"$gte": query_start_timestamp}}, {'_meta.inserted_at': {"$lt": query_end_timestamp}}]}            

                    
                    documents =self.collection_people_perception.find(query).sort('_meta.inserted_at',pymongo.ASCENDING).limit(1000000)
                    
                    print "query:", query
                    length_documents=documents.count()
                    print "Found documents:", length_documents
    
                    current_humans=[]
                    last_uuid=None
                    
                    for idocuments in documents:    
                       for j in idocuments['people_tracker']['uuids']:
                            if j not in current_humans:
                                current_humans.append(j)


                    for ihuman in current_humans:
                        
                        self.trajectory_person(ihuman,query_start_timestamp,query_end_timestamp)

    def trajectory_person(self,currentuuid,start_date,end_date):
        
        
        
        query_human ={"$and": [ {'_meta.inserted_at': {"$gt": start_date}},{'_meta.inserted_at': {"$lt": end_date}},{"uuids":{"$in": [currentuuid]}},{'current_edge':'none'} ]}
                        
        
        documents_humans =self.collection_people_perception.find(query_human).sort('_meta.inserted_at',pymongo.ASCENDING).limit(50000)       
        length_document = documents_humans.count()
        print "Found length_documents_humans:", length_document
        #print 'uuid=',documents_humans[0]['closest_node']
        if length_document > 0:    
            init_timestamp = documents_humans[0]['header']['stamp']['secs']
            end_timestamp = documents_humans[length_document-1]['header']['stamp']['secs']
            waypoint=documents_humans[0]['closest_node']
            robot_pose=documents_humans[0]['robot']
            print 'init_timestamp=',init_timestamp
            print 'end_timestamp=',end_timestamp
            total_time=float( end_timestamp-init_timestamp)
            print 'total_time=',total_time
            

            if self.min_time_detected <= total_time <= self.max_time_detected :
                #print 'ENTRY'
                #entries.append(new_entry)
                
                poses=[]
                
                for idocuments_humans in documents_humans:
                    
                    index_uuid=0
                    print 'documents_humans uuids',idocuments_humans['people_tracker']['uuids']
                    
                    #index_uuid=idocuments_humans['people_tracker']['uuids'].index(currentuuid)
                    for index,iuuid in enumerate( idocuments_humans['people_tracker']['uuids']):
                        if currentuuid == iuuid:
                            index_uuid=index
                    
                    angle=idocuments_humans['people_tracker']['angles'][index_uuid]
                    distance=idocuments_humans['people_tracker']['distances'][index_uuid]
                    posetimestamp=idocuments_humans['people_tracker']['header']['stamp']['secs']
                    
                    pose={"timestamp":posetimestamp,"angle":angle, "distance":distance}
                    poses.append(pose)
            
            
                
                new_entry = {
                "uuid" : currentuuid,
                "init_timestamp" : init_timestamp,
                "end_timestamp" : end_timestamp,
                "total_time" : total_time,
                "waypoint" : waypoint,
                "robot_pose": robot_pose,
                "person_poses":poses
                }
                
                #print 'new_entry=',new_entry
                self.collection_trajectory_person.insert(new_entry)
                
                print 'SAVED date=',start_date.strftime("%Y/%m/%d"),'end_timestamp=',end_timestamp

                    
    def advanced_filter(self):
        print 'advanced_filter'
        documents_people_tracker = self.collection_trajectory_person.find().sort("$natural",pymongo.DESCENDING).limit(50000) 
        print "Found documents_people_tracker:", documents_people_tracker.count()
        lenght=documents_people_tracker.count()
        count=0
        

        for idocuments_people_tracker in documents_people_tracker:
            
            print 'count =',count,' (',lenght,')'
            uuid=idocuments_people_tracker['uuid']
            iddoc=idocuments_people_tracker['_id']
            
            poses=idocuments_people_tracker['person_poses']
            distance_total=0.0
            last_angle=float(poses[0]['angle'])
            last_dist=float(poses[0]['distance'])
            for ipose in poses:
                
                actual_angle=float(ipose['angle'])
                actual_dist=float(ipose['distance'])

                dist=float(math.pow(last_dist, 2)+math.pow(actual_dist, 2))-float(2.0*last_dist*actual_dist*math.cos(actual_angle-last_angle))
                #print 'dist=',dist
                if dist> 0.01:
                    dist_sqrt=math.sqrt(dist)
                    distance_total +=abs(dist_sqrt)
   
                last_angle=actual_angle
                last_dist=actual_dist
                #print 'dist= ',dist
               
            if distance_total > 0.2:
                
                new_entry = {
                "uuid" : idocuments_people_tracker['uuid'],
                "init_timestamp" : idocuments_people_tracker['init_timestamp'],
                "end_timestamp" : idocuments_people_tracker['end_timestamp'],
                "total_time" : idocuments_people_tracker['total_time'],
                "waypoint" : idocuments_people_tracker['waypoint'],
                "robot_pose": idocuments_people_tracker['robot_pose'],
                "person_poses":idocuments_people_tracker['person_poses']
                }
                self.collection_trajectory_person_filtered.insert(new_entry)
                print 'SAVED'
            count+=1


if __name__ == '__main__':
    
    
    rospy.init_node('users_analysis')
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--datapath", type=str, default='./data',
                        help="Data folder.  default='./data'.")
    parser.add_argument("--videofolders", type=str, default='head_camera_2017-01,head_camera_2017-02,head_camera_2017-03',
                        help="List of video folders separated by ',' in <datapath>/videos.  default='head_camera_2017-01,head_camera_2017-02'.")                        
    parser.add_argument("--advancedfilter", type=bool, default=False,
                        help="Advanced filter discard static detections default= false")  
                    
    args = parser.parse_args()
    

    path_videos=args.datapath+'/videos'
    temp_video_folders=args.videofolders
    try:
        path_video_folders=temp_video_folders.split(',')
    except:
        path_video_folders[0]=temp_video_folders
        
    advancedfilter=args.advancedfilter
    
    mongo= MongoDb(path_videos,path_video_folders,advancedfilter)
    

