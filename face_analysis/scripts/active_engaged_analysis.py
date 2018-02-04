#!/usr/bin/python
'''
----------------------------------
USERS ANALYSIS
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

import opencv_tools

from threading import Thread
import json
import pymongo 
import datetime
import cv2
import collections
import glob




def loadjson(path):
    print "load " ,path   
    # Reading data
    with open(path, 'r') as f:
        data = json.load(f)

    return data

class MongoDb():
    

        
    def __init__(self,path_folder_output,path_videos,video_folders) :
        counts=[]
        self.detections=[]
        self.mode_analysis= 'auto' # 'auto', 'manual'
        self.cvframe=opencv_tools.ExtractFrame()
        try:
            self.client = pymongo.MongoClient('localhost', 62345)
            
            #self.client = pymongo.MongoClient('10.5.42.18', 23456)                      
            self.db_analysis = self.client['final_users_analysis']                       
            self.collection_interactions = self.db_analysis['active_interactions']
            self.collection_trajectory_person= self.db_analysis['trajectory_person']
            self.collection_users= self.db_analysis['active_users']
            self.msgstore=True
        except:
            self.msgstore=False
            print ("Message Store not available")   


        self.videofolder=None
        self.videopath=None
        
        
        for self.videofolder in video_folders:
            

            with open(path_videos+'/info_videos_'+self.videofolder+'.json', 'r') as f:
                videodata = json.load(f)
                
            for ivideo in videodata['video']:
    
                if ivideo['name']!='':
                    
                    name_file=ivideo['name']
                                   
                    temp_name=name_file.split('_')
                    date=temp_name[2].split('-')
                    tmp=temp_name[3].split('.')
                    time=tmp[0].split('-')
                    
                    name_path=path_videos+'/'+self.videofolder

                    print 'name_path',name_path
                    print 'name_file',name_file
                    
                    self.cvframe.load_info_video(path_videos,self.videofolder,name_file)

                    print 'ExtractFrame'
                    
                    start_timestamp=int(self.cvframe.array_timestamps[0]['timestamp'])
                    end_timestamp=int(self.cvframe.array_timestamps[len(self.cvframe.array_timestamps)-1]['timestamp'])
    
                    #start_timestamp=1483282169                
    
                    query ={"$and": [ {'timestamp': {"$gte": start_timestamp}},{'timestamp': {"$lt": end_timestamp}}]}                
    
                            
                    
                    documents_interaction =self.collection_interactions.find(query).sort('timestamp',pymongo.ASCENDING).limit(5000)
                    print "query:", query
                    length_documents=documents_interaction.count()
                    print "Found documents:", length_documents
    
                    count_idocument=0
                    for idocuments in documents_interaction:
                        current_timestamp=int(idocuments['timestamp'])
                        
                        print 'TIMESTAMP:: ',current_timestamp
                        print 'count_idocument:: ',count_idocument, '(', length_documents,')'
                        
                        ### FACES
                        
                        b_ms_face=False 
                        
                        opencvfaces,current_videoframe=self.cvframe.detect_faces(current_timestamp,1,1) #(current_timestamp,framesnumber=1,increment=1) 
    
                                       
                        if len(opencvfaces)>0:
                            
                            image,current_videoframe=self.cvframe.rosimage(current_videoframe,current_timestamp)
                            ret_face=ms_face_call(image,'','',True)
                            if ret_face.faces.faces>0:
                                b_ms_face=True
                                
                        if b_ms_face is False:
                            
                            print 'searching for opencvfaces below'
                            opencvfaces,current_videoframe=self.cvframe.detect_faces(current_timestamp,3,-1)
    
                            if len(opencvfaces)>0:
                                
                                rospy.sleep(2.0)
                                ret_face,current_videoframe=self.cvframe.rosimage(current_videoframe,current_timestamp)
                                ret_face=ms_face_call(image,'','',True)
                                if ret_face.faces.faces>0:
                                    b_ms_face=True
                                    
                                    
                            if b_ms_face is False:
                                print 'searching for opencvfaces above'
                                opencvfaces,current_videoframe=self.cvframe.detect_faces(current_timestamp,3,1)
                                
                                if len(opencvfaces)>0:
                                    rospy.sleep(2.0)
                                    ret_face,current_videoframe=self.cvframe.rosimage(current_videoframe,current_timestamp)
                                    ret_face=ms_face_call(image,'','',True)
                                    if ret_face.faces.faces>0:
                                        b_ms_face=True
                                    
                                    
                                    
                                    
                        if b_ms_face is True:
                            
                            
                            face_timestamp=self.cvframe.estimate_timestamp(current_videoframe)
    
                            n_users=0
                            
    
                            # users ordered by biggest face area
                                
                            max_area = 0.0
                            list_faces = {}
                            sorted_users=[]
                            for f in ret_face.faces.faces: 
                                area = float(f.faceRectangle.width * f.faceRectangle.height)
                                #l_f={'area':area, 'name':f.person}
                                list_faces[area] =   f.person
                                #list_faces.append(l_f)
                            
                            sorted_faces= collections.OrderedDict(sorted(list_faces.items()), reverse=True)
                            
                            
                            for k,v in list_faces.iteritems():
                
                                sorted_users.append(v)
                                
                            print 'sorted_users=',sorted_users
    
                            n_users=len(ret_face.faces.faces)
    
                            ### end order                        
                       
                            for face in ret_face.faces.faces:
                                
                                
                                username=face.person
                                
                                print 'username',username
                                appearance=json.loads(face.faceAttributes)
                                demographics={'gender':face.gender,'age':face.age}
                                confidence=face.confidence
                                smile=face.smile
                                
                                primary_user=False
                                if username==sorted_users[0]: 
                                    primary_user=True
    
                                new_interaction = {
                                'confidence':confidence,
                                'timestamp':face_timestamp,
                                'asociated_inter':idocuments['_id'],
                                "interaction-time" : 0,
                                "screen" : idocuments['screen'],
                                "waypoint" : idocuments['waypoint'],
                                "demographics": demographics,
                                "appearance": appearance,
                                "mood": smile,
                                "n-users":n_users,
                                "primary-user":primary_user
                                }
                                
                                interactions=[]
                                interactions.append(new_interaction)
                                
                                
                                    
                                if confidence == -1.0:  #NEW USER
                                    print 'new user'
    
                                    new_user={'user-ID': username, 'interaction':interactions} 
    
                                    self.collection_users.insert(new_user)
                                    
                                else:   #OLD USER NEW INTERACTION
                                
                                    try:
                                        self.collection_users.update({'user-ID': username}, {'$push': {'interaction': new_interaction}})
                                        continue
                                    except:
                                        print 'Impossible to update user-ID: ',username
                           
                           
                                self.cvframe.save_roi(current_videoframe,face.faceRectangle,username,current_timestamp,path_folder_output)
                            
                           
                            iddoc=idocuments['_id']
                            
                            media_face={'videoname':self.cvframe.videoname, 'timestamp':face_timestamp, 'numframe': current_videoframe } 
                                
                                
                            self.collection_interactions.update({'_id' : iddoc}, {'$set': {'faces': sorted_users,'media': media_face}}, upsert=False, multi=True)
                            
                            
                            #### POSSIBLE  UUIDS
                            
                
                            documents_uuids =self.collection_trajectory_person.find({"$and": [ {'init_timestamp': {"$lte": current_timestamp}},{'end_timestamp': {"$gt": current_timestamp}} ]}).sort('init_timestamp',pymongo.ASCENDING)
                            print "Found documents_trajectories:", documents_uuids.count()     
                            
                            uuids=[]
                            for idocuments_uuids in documents_uuids:
                                uuids.append(idocuments_uuids['uuid'])
                                
                            print 'uuids=',uuids        
                            self.collection_interactions.update({'_id' : iddoc}, {'$set': {'uuids': uuids}}, upsert=False, multi=True)
    
            
            
                                    
                        count_idocument+=1              
                        
            
                        print '_____________________'
                        print 'waiting'
                        rospy.sleep(2.0)




                
                
       
    

if __name__ == '__main__':
    
    ms_face_call=rospy.ServiceProxy('/cognitivefaceapi/detect',Detect)
    
    rospy.init_node('active_engaged_analysis')
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--datapath", type=str, default='./data',
                        help="Data folder.  default='./data'.")
    parser.add_argument("--videofolders", type=str, default='head_camera_2017-01,head_camera_2017-02,head_camera_2017-03',
                        help="List of video folders separated by ',' in <datapath>/videos.  default='head_camera_2017-01,head_camera_2017-02'.")                        
 
    args = parser.parse_args()
    


    path_output = args.datapath+'/output_data/users_active_engaged'
    path_videos=args.datapath+'/videos'
    temp_video_folders=args.videofolders
    try:
        path_video_folders=temp_video_folders.split(',')
    except:
        path_video_folders[0]=temp_video_folders
        
    print 'video_folders=',path_video_folders
    print "Started"
    mongo= MongoDb(path_output,path_videos,path_video_folders)
    


