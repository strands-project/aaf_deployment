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

path_folder_input='/media/robpin/063CAF573CAF4113/LCAS/collect_data_trajectories/data/passive_interactions_manually'
path_folder_passive_confirmed='/media/robpin/063CAF573CAF4113/LCAS/collect_data_trajectories/data/passive_confirmed'
path_folder_output='/media/robpin/063CAF573CAF4113/LCAS/collect_data_trajectories/data/new_passive_engaged'
path_videos='/media/robpin/063CAF573CAF4113/LCAS/collect_data_trajectories/videos'

defaul_folder_camera='head_camera_2017-01'
default_video='head_camera_2017-01-04_06-00-01.avi'

video_folders=['head_camera_2017-01','head_camera_2017-02','head_camera_2017-03']

haarcascade_path='/home/robpin/catkin_ws/src/trajectory_analysis_gui/haarcascades'

class MongoDb():
    

        
    def __init__(self) :
        counts=[]
        self.detections=[]
        
        self.cvframe=ExtractFrame()
        self.list_uuids=[]
        self.index_uuids=-1
        self.list_queries=[]
        self.index_queries=0 #-1
        self.length_queries=0
        self.cvframe=opencv_tools.ExtractFrame()
        self.cvframe.arrayvideofolders=video_folders
        try:
            self.client = pymongo.MongoClient('localhost', 62345)
            
            #self.client = pymongo.MongoClient('10.5.42.18', 23456)                      
            self.db = self.client['final_users_analysis']                        
            self.collection_passive_interactions = self.db['passive_interactions']
            self.collection_trajectory_person= self.db['trajectory_person_filtered']
            self.collection_passive_users= self.db['passive_users']

            self.collection_frongo= self.db['passive_users_frongo']
            self.db_message_store = self.client['message_store_y4']
            self.info_terminal = self.db_message_store['info_terminal_active_screen']
            self.msgstore=True
        except:
            self.msgstore=False
            print ("Message Store not available")   
            
        
        self.end_numframe=10
        self.waiting=True
        
        



    def auto_analysis(self):

        documents_trajectories = self.collection_trajectory_person.find().sort("$natural",pymongo.DESCENDING).limit(50000) 
        print "Found documents_trajectories:", documents_trajectories.count()
                

        
        for idocuments_trajectories in documents_trajectories:

            current_uuid=idocuments_trajectories['uuid']
            print "uuid:", current_uuid
                                    
            iddoc=idocuments_trajectories['_id']
            

            self.cvframe.video_from_timestamp(idocuments_trajectories['init_timestamp'],video_folders)
            
            interaction='non-interaction'
            
            faces_array=[]
            last_timestamp=0
      
            for ipose in idocuments_trajectories['poses']:
                
                current_timestamp=ipose['timestamp']
                
                angle=ipose['angle']
                
                if current_timestamp !=last_timestamp and (angle > 2.0 or angle< -2.0 ):  
                                                
                    #print 'current_timestamp=',current_timestamp
  
                    faces=self.cvframe.detect_faces(current_timestamp,1,1)
                    
                    if len(faces)>0:
                        #print 'faces',faces
                       
                        # corresponding face - uuid
                        list_faces=[]
                        iface=0
                        for face in faces:
                            
                            x= int(face[0])
                            y= int(face[1])
                            w = int(face[2])
                            h = int(face[3])
                            
                            center_facex=x+int(w/2)
                            center_facey=y+int(h/2)
                            
                            # pixels to angle
                            image_width_pixels=640
                            fov_radians =0.95 # 0.87 =50 degrees
                            f = ( image_width_pixels / 2.0 ) / math.tan( fov_radians / 2.0 )
                            angle_radians = math.atan( (center_facex-(image_width_pixels/2) )/f )
                            #print 'angle_radians1=',angle_radians
                            
                            
                            if angle_radians >0:
                                angle_radians=  math.pi - angle_radians
                                #print 'math.pi=',math.pi
                            else:
                                angle_radians= -math.pi - angle_radians
                                
                            print 'angle face=',angle_radians
                            print 'angle people_detection=',angle
                            dist=abs(angle_radians-angle)
                            new_face={'index':iface, 'dist_angle':dist}
                            list_faces.append(new_face)
                            iface+=1
                            
                        
                        closest_angle=math.pi
                        index_faces=None
                        for i,face in enumerate(list_faces):
                            if float(face['dist_angle']) < closest_angle:
                                closest_angle=float(face['dist_angle'])
                                index_faces=i

                        
                        print 'closest_angle=',closest_angle
                        max_dist_face=0.2                                   
                        if  closest_angle < max_dist_face:
                            
                            x= int(faces[index_faces][0])
                            y= int(faces[index_faces][1])
                            w = int(faces[index_faces][2])
                            h = int(faces[index_faces][3])
                            
                            face_detection={'xmin':x,'ymin':y,'width':w,'height':h,'timestamp':current_timestamp}
                            faces_array.append(face_detection)
                    
                            interaction='passive-interaction'
                            
                 
                last_timestamp=current_timestamp
           
       
            print 'END FOR date=',name_file
        
            if len(faces_array)>0:
                
                currentscreen=self.getscreen(faces_array[0]['timestamp'])
                
                new_interaction = {
                'faces':faces_array,
                "screen" : currentscreen,
                "waypoint" : idocuments['waypoint'],
                "uuid" : current_uuid,
                'interaction-type': interaction
                }

        
                self.collection_passive_interactions.insert(new_interaction)

                print 'SAVED date=',name_file,'uuid=',current_uuid


    def load_list(self):

        query={ 'interaction-type-labeled': { "$exists": False }}
        
        documents_trajectories = self.collection_passive_interactions.find(query).sort("$natural",pymongo.DESCENDING).limit(50000) 
        print "Found documents_trajectories:", documents_trajectories.count()

        for idocuments_trajectories in documents_trajectories:        

            self.list_uuids.append(idocuments_trajectories['uuid'])        
            
    def setnewuser(self):
        
        self.index_uuids+=1
        
        uuid=self.list_uuids[self.index_uuids]
        
        print '#####'
        print 'uuid=',uuid,'####'

        print 'self.index_uuids=', self.index_uuids
        print 'total uuids=',len(self.list_uuids)
        
        
        self.list_queries =self.collection_trajectory_person.find({'uuid': str(uuid)}).sort('init_timestamp',pymongo.ASCENDING)
        length_documents=self.list_queries.count()
        print "Found documents:", length_documents
        
        idocuments_trajectories=self.list_queries[0]   
        
        print 'init_timestamp=',idocuments_trajectories['init_timestamp']
        videopath=self.cvframe.video_from_timestamp(idocuments_trajectories['init_timestamp'])
        
        init_numframe=int(self.cvframe.estimate_frame(idocuments_trajectories['init_timestamp']))
        end_numframe=int(self.cvframe.estimate_frame(idocuments_trajectories['end_timestamp']))
 
        return videopath,init_numframe,end_numframe

    def getpose(self,currentframe):
        
        poses=self.list_queries[self.index_queries]['poses']
        angle=0.0
        distance=0.0

        for ipose in poses:
            numframe_pose=int(self.cvframe.estimate_frame(ipose['timestamp']))


            if numframe_pose==currentframe:
                angle=ipose['angle']
                distance=ipose['distance']
        return angle,distance                
        
    def savedata(self,frame,interactionnumframe,typeinteraction,roi):
        
        idocuments_trajectories=self.list_queries[self.index_queries]
        iddoc=idocuments_trajectories['_id']
        uuuidname=idocuments_trajectories['uuid']
        
        
        
        faces=[]
        print 'typeinteraction=',typeinteraction
        str_interaction=''
        if typeinteraction ==0:
            str_interaction='non-interaction'
        elif typeinteraction ==1:
            str_interaction='passive-interaction'
        elif typeinteraction ==2:
            str_interaction='active-interaction'
        elif typeinteraction ==3:
            str_interaction='non-person'

            
        ##'non-interaction' =0 'non-person' =3
        if typeinteraction !=0 and typeinteraction !=3:
            timestamp=self.cvframe.estimate_timestamp(interactionnumframe)
            x= int(roi[0])
            y= int(roi[1])
            w = int(roi[2])
            h = int(roi[3])
                            
            userdata={ 'xmin':x,'ymin':y,'width':w,'height':h,'timestamp': timestamp, 'user-ID':''}
            faces.append(userdata)
        

        self.collection_passive_interactions.update({'_id' : iddoc}, {'$set': {'interaction-type-labeled':str_interaction, 'faces':faces}}, upsert=False, multi=True)
        
        
        #### SAVE  ROI
        
        if typeinteraction !=0 and typeinteraction !=3:

            self.cvframe.save_roi(frame,interactionnumframe,roi,uuuidname,timestamp,typeinteraction)
            
        
    def getscreen(self,current_timestamp):
        
        currentscreen=''
        #datetime.datetime(2011, 10, 4, 16, 46, 59, 786000)
        query_time=datetime.datetime.fromtimestamp(current_timestamp)
        query_time=query_time+ datetime.timedelta(hours=1)
        ##### SEARCH THE CLOSEST CHANGE ####
        closestBelow = self.info_terminal.find({"$and": [ {'_meta.inserted_at': {"$lte": query_time }}]}).sort('_meta.inserted_at',pymongo.DESCENDING).limit(1)

        length_closestBelow=closestBelow.count()
        print "Found documents:", length_closestBelow
        if length_closestBelow>0:
            print "closestBelow time : ", closestBelow[0]['_meta']['inserted_at']
            print "closestBelow screen: ", closestBelow[0]['data']
            currentscreen=closestBelow[0]['data']
            
        return currentscreen

    def getfaceuser(self):
        

        documents=self.collection_passive_interactions.find().limit(1000)
        length_documents=documents.count()
        print "Found documents:", length_documents
        
        index_person=0
        for idocuments in documents:
            iddoc=idocuments['_id']
            current_timestamp=int(idocuments['faces'][0]['timestamp'])
            
            uuid=idocuments['uuid']
            
            facearray=idocuments['faces']


            print 'uuid: ',index_person,'from total:',length_documents
            index_person+=1

#            if index_person <241:
#                continue
            
            if idocuments['faces'][0]['user-ID']!='':
                
                print 'break'
                continue
            
            username=''
            

            #### MS FACE API
            name_pic=str(current_timestamp)
            

            print 'name_pic',name_pic

            imagefile=path_folder_input+'/'+uuid+'/'+name_pic+'.png'
            print 'imagefile=',imagefile
            image=Image()
            
            try:
                resp=ms_face_call(image,imagefile,'',True)  #MS FACE API
                print 'resp=',resp
                
                for face in resp.faces.faces:
                    
                    username=face.person
                    
                    print 'username',username
                    appearance=json.loads(face.faceAttributes)
                    demographics={'gender':face.gender,'age':face.age}
                    confidence=face.confidence
                    smile=face.smile
                    
#                    # color with max confidence
#                    index_def_hair=None
#                    max_conf=0.0
#                    for i in range(len(hair['hairColor'])):
#                        if hair['hairColor'][i]['confidence']>max_conf:                        
#                            max_conf=hair['hairColor'][i]['confidence']
#                            index_def_hair=i
#                            
#                    def_hair=hair['hairColor'][i]
#                    if hair['bald']>0.7:
#                        def_hair['color']='bald'
#                        def_hair['confidence']=hair['bald']


 

            except:
                pass
            
            imageframe=path_folder_input+'/'+uuid+'/frame_'+name_pic+'.png'
            videopath=self.cvframe.video_from_timestamp(current_timestamp)
            opencvfaces=self.cvframe.detect_faces_image(imageframe) #(current_timestamp,framesnumber=1,increment=1
            
            n_users=len(opencvfaces)
            
            facearray[0]['user-ID']=username
            
            
            if username!='':

                # INTERACTION TIME
                documents_uuids =self.collection_trajectory_person.find({'uuid': str(uuid)}).sort('init_timestamp',pymongo.ASCENDING)
                length_documents_uuids=documents_uuids.count()
                print "Found documents:", length_documents_uuids
                
                idocuments_trajectories=documents_uuids[0]   
                
                interaction_time=idocuments_trajectories['end_timestamp']-idocuments_trajectories['init_timestamp']
                

                new_interaction = {
                'confidence':confidence,
                'timestamp':current_timestamp,
                'asociated_inter':iddoc,
                "interaction-time" : interaction_time,
                "screen" : idocuments['screen'],
                "waypoint" : idocuments['waypoint'],
                "demographics": demographics,
                "appearance": appearance,
                "mood": smile,
                "n-users":n_users,
                "primary-user":False
                }
                
                interactions=[]
                interactions.append(new_interaction)
                

                new_user={'user-ID': username, 'interaction':interactions} 

                self.collection_passive_users.insert(new_user)
                                    
                face_array=[]


                self.collection_passive_users.update({'_id' : iddoc}, {'$set': {'faces': facearray}}, upsert=False, multi=True)
                print 'SAVED  waiting...'
                
            time.sleep(2.0)


class ShowCapture(wx.Frame):
    def __init__(self, capture, fps=8):
        wx.Frame.__init__(self, None)
        panel = wx.Panel(self, -1)

        #create a grid sizer with 5 pix between each cell
        sizer = wx.GridBagSizer(5, 5)

        self.capture = capture
        ret, self.orig_frame = self.capture.read()

        height, width = self.orig_frame.shape[:2]
        self.orig_height = height
        self.orig_width = width

        self.orig_frame = cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB)

        self.bmp = wx.BitmapFromBuffer(width, height, self.orig_frame)

        self.dummy_element = wx.TextCtrl(panel, -1,'')
        self.dummy_element.Hide()

        #create SpinCtrl widgets, these have vertical up/down buttons and a TextCtrl that increments with up/down press
        self.roi_x = wx.SpinCtrl(panel, -1, "ROI X",  style=wx.TE_PROCESS_ENTER|wx.SP_ARROW_KEYS, min=0, max=width, initial=0, size=(60,-1))
        self.roi_y = wx.SpinCtrl(panel, -1, "ROI Y",  style=wx.TE_PROCESS_ENTER|wx.SP_ARROW_KEYS, min=0, max=height, initial=0, size=(60,-1))
        self.roi_width = wx.SpinCtrl(panel, -1, "ROI W",  style=wx.TE_PROCESS_ENTER|wx.SP_ARROW_KEYS, min=0, max=width, initial=width, size=(60,-1))
        self.roi_height = wx.SpinCtrl(panel, -1, "ROI H",  style=wx.TE_PROCESS_ENTER|wx.SP_ARROW_KEYS, min=0, max=height, initial=height, size=(60,-1))

        save_bmp_path = os.path.join(os.path.dirname(__file__), 'icons', 'ic_action_save.png')
        if os.path.isfile(save_bmp_path):
            save_bmp = wx.Image(save_bmp_path).ConvertToBitmap()
            save_button = wx.BitmapButton(panel,wx.ID_ANY, bitmap=save_bmp, style = wx.NO_BORDER, size=(32,32)) # )
        else:
            save_button = wx.Button(panel, -1, 'Save')
            
        
        newuser_button = wx.Button(panel, -1, 'New user')
        play_button = wx.Button(panel, -1, 'Play')
        pause_button = wx.Button(panel, -1, 'Capture')
        replay_button = wx.Button(panel, -1, 'Replay')

        #create image display widgets
        self.ImgControl = statbmp.GenStaticBitmap(panel, wx.ID_ANY, self.bmp)
        self.ImgControl2 = statbmp.GenStaticBitmap(panel, wx.ID_ANY, self.bmp)

        #add text to the sizer grid
        sizer.Add(wx.StaticText(panel, -1, 'ROI'), (0, 4), (1,5),  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'X'), (1, 4), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'Y'), (2, 4), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'width,'), (1, 6), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'height'), (2, 6), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'Right-click image to reset ROI'), (6, 8), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'Key N : New user'), (1, 8), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'Key 0 : Save data'), (2, 8), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'Key LEFT :  Replay'), (3, 8), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'Key DOWN : Capture'), (4, 8), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(wx.StaticText(panel, -1, 'Key RIGTH: Play'), (5, 8), wx.DefaultSpan,  wx.ALL, 5)
        
        tool_button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        tool_button_sizer.Add(newuser_button, 0)
        tool_button_sizer.Add(save_button, 0)
                
        
        tool_button_sizer2 = wx.BoxSizer(wx.HORIZONTAL)
        tool_button_sizer2.Add(replay_button, 1)
        self.fpsreplay = wx.SpinCtrl(panel, -1, "FPS REPLAY",  style=wx.TE_PROCESS_ENTER|wx.SP_ARROW_KEYS, min=1, max=30, initial=4, size=(60,-1))
        tool_button_sizer2.Add(self.fpsreplay, 0)
        tool_button_sizer2.Add(wx.StaticText(panel, -1, 'fps'), 0)
        tool_button_sizer2.Add(play_button, 0)
        tool_button_sizer2.Add(pause_button, 0)



        tool_button_sizer3 = wx.BoxSizer(wx.HORIZONTAL) 
        sampleList = ['non-interaction','passive-interaction','active-interaction', 'non-person']
        self.interType=wx.Choice(panel, -1, (85, 18), choices=sampleList)

        tool_button_sizer3.Add(wx.StaticText(panel, -1, "\t Interaction type:", (15, 20)), 0)
        tool_button_sizer3.Add(self.interType, 0)
        
        
        #sizer.Add(, (0, 6), wx.DefaultSpan, wx.ALIGN_RIGHT)#,  wx.ALL, 5)
        sizer.Add(tool_button_sizer, (0, 1), wx.DefaultSpan, wx.ALIGN_LEFT)#,  wx.ALL, 5)
        sizer.Add(tool_button_sizer2, (1, 1), wx.DefaultSpan, wx.ALIGN_LEFT)
        sizer.Add(tool_button_sizer3, (2, 1), wx.DefaultSpan, wx.ALIGN_LEFT)
        
        #add SpinCtrl widgets to the sizer grid
        sizer.Add(self.roi_x, (1, 5), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(self.roi_y, (2, 5), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(self.roi_width, (1, 7), wx.DefaultSpan,  wx.ALL, 5)
        sizer.Add(self.roi_height, (2, 7), wx.DefaultSpan,  wx.ALL, 5)

        #add image widgets to the sizer grid
        sizer.Add(self.ImgControl, (3, 0), (1,4), wx.EXPAND|wx.CENTER|wx.LEFT|wx.BOTTOM, 5)
        sizer.Add(self.ImgControl2, (3, 4), (1,4), wx.EXPAND|wx.CENTER|wx.RIGHT|wx.BOTTOM, 5)

        #set the sizer and tell the Frame about the best size
        panel.SetSizer(sizer)
        sizer.SetSizeHints(self)
        panel.Layout()
        panel.SetFocus()

        #start a timer that's handler grabs a new frame and updates the image widgets
        self.timer = wx.Timer(self)
        self.fps = fps
        self.timer.Start(1000./self.fps)

        #bind timer events to the handler
        self.Bind(wx.EVT_TIMER, self.NextFrame)
        #self.Bind(wx.EVT_TIMER, self.ExtractROI)
        
        self.pause=True

        
        
        #bind events to ROI image widget
        self.ImgControl2.Bind(wx.EVT_LEFT_DOWN, self.On_ROI_Click)
        self.ImgControl2.Bind(wx.EVT_LEFT_UP, self.On_ROI_ClickRelease)
        self.ImgControl2.Bind(wx.EVT_RIGHT_DOWN, self.On_ROI_RightClick)
        self.ImgControl2.Bind(wx.EVT_MOTION, self.On_ROI_Hover)
        self.ImgControl2.Bind(wx.EVT_ENTER_WINDOW, self.On_ROI_mouse_enter)
        self.ImgControl2.Bind(wx.EVT_LEAVE_WINDOW, self.On_ROI_mouse_leave)


        #bind  buttons
        newuser_button.Bind(wx.EVT_BUTTON, self.on_newuser_click)
        play_button.Bind(wx.EVT_BUTTON, self.on_play_click)
        pause_button.Bind(wx.EVT_BUTTON, self.on_pause_click)
        replay_button.Bind(wx.EVT_BUTTON, self.on_replay_click)
        
        #bind save button
        save_button.Bind(wx.EVT_BUTTON, self.on_save_click)
        save_button.Bind(wx.EVT_RIGHT_DOWN, self.on_quick_save)

        #bind settings button

        #settings_button.Bind(wx.EVT_LEFT_UP, self.on_settings_click_release)

        self.Bind(wx.EVT_KEY_UP, self.on_key_up)
        
        self.mongo=MongoDb()
        self.mongo.load_list()


        self.init_numframe=0
        self.end_numframe=10
        self.current_numframe=0
        self.captured_numframe=-1
        self.capturedframe=None


    def on_key_up(self, evt):
        
        keycode = evt.GetKeyCode()
        print keycode
        if keycode == 78: # N
            self.on_newuser_click(evt)
        if keycode == 324: # 0
            self.on_save_click(evt)
        if keycode == 314: # N
            self.on_replay_click(evt)            
        if keycode == 316: # N
            self.on_play_click(evt)            
        if keycode == 317: # N
            self.on_pause_click(evt)            
#        if keycode == wx.WXK_SPACE:
#            print "you pressed the spacebar!"
        evt.Skip()

    def on_settings_click(self, evt):
        
        print 'on_settings_click'
        


    def on_settings_click_release(self, evt):
        self.dummy_element.SetFocus()

    def on_newuser_click(self,evt):
        

        videocapture,self.init_numframe,self.end_numframe=self.mongo.setnewuser()
        self.capture=cv2.VideoCapture(videocapture)
        self.current_numframe=self.init_numframe
        self.captured_numframe=-1


        
        self.timer.Stop()
        self.timer.Start(1000./self.fps)
        
        self.pause=False
            
    def on_play_click(self, evt):

        #self.timer.Start(1000./self.fps)
        self.pause=False

    def on_pause_click(self, evt):
        
        self.roi_x.SetValue(0)        
        self.roi_y.SetValue(0)
        self.roi_width.SetValue(self.orig_width)
        self.roi_height.SetValue(self.orig_height)
        
        self.captured_numframe=self.current_numframe
        self.capturedframe=self.orig_frame
        self.pause=True
        
        #self.ImgControl2.SetBitmap(self.bmp)
        
    def on_replay_click(self, evt):
        
        self.current_numframe=self.init_numframe

        self.timer.Stop()
        self.timer.Start(1000./float(self.fpsreplay.GetValue()))
        
        self.pause=False
        
        
    def on_save_click(self, evt):
    
        self.mongo.savedata(self.capturedframe,self.captured_numframe,self.interType.GetSelection(),self.frameRoi)

    def on_quick_save(self, evt):
        cv2.imwrite('orig_frame.jpg', self.orig_frame)
        cv2.imwrite('frameRoi.jpg', self.frameRoi)

    def On_ROI_RightClick(self, evt):
        self.roi_x.SetValue(0)        
        self.roi_y.SetValue(0)
        self.roi_width.SetValue(self.orig_width)
        self.roi_height.SetValue(self.orig_height)

    def On_ROI_Hover(self, evt):
        self.ROI_crosshair_pos = evt.GetPosition()

    def On_ROI_mouse_enter(self, evt):
        self.enable_crosshairs = True

    def On_ROI_mouse_leave(self, evt):
        try:
            self.enable_crosshairs = False
            if hasattr(self, 'roi_click_down_pos'):
                self.update_spinners(evt.GetPosition())
            del self.roi_click_down_pos
        except AttributeError:
            pass

    def On_ROI_Click(self, evt):
        self.roi_click_down_pos = evt.GetPosition()

    def On_ROI_ClickRelease(self, evt):
        roi_click_up_pos = evt.GetPosition()
        #if roi_click_up_pos[0] >= 0 and roi_click_up_pos[1] >= 0:
        if hasattr(self, 'roi_click_down_pos'):
            self.update_spinners(roi_click_up_pos)
        try:
            del self.roi_click_down_pos
        except AttributeError:
            pass

    def update_spinners(self, new_pos):
        self.roi_width.SetValue(abs(new_pos[0] - self.roi_click_down_pos[0]))
        self.roi_height.SetValue(abs(new_pos[1] - self.roi_click_down_pos[1]))
        self.roi_x.SetValue(min(self.roi_click_down_pos[0], new_pos[0]))
        self.roi_y.SetValue(min(self.roi_click_down_pos[1], new_pos[1]))

    def NextFrame(self, event):
        
        if self.pause is False:

            if self.current_numframe<self.end_numframe:
                    
                self.capture.set(1,self.current_numframe); # Where numframe is the frame you want
                ret, self.orig_frame  = self.capture.read() # Read the frame
                if ret:
                    self.currentframe = cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB)
                    
                    frame=self.DrawUserPosition(self.currentframe)


                    self.bmp.CopyFromBuffer(frame)
                    self.ImgControl.SetBitmap(self.bmp)
                self.current_numframe+=1
                
#            ret, self.orig_frame = self.capture.read()
#            if ret:
#                self.currentframe = cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB)
#    
#                self.bmp.CopyFromBuffer(self.currentframe)
#                self.ImgControl.SetBitmap(self.bmp)
        else: #PAUSED
            self.ExtractROI(event)            


    def DrawUserPosition(self,frame):
        

        angle_radians,dist=self.mongo.getpose(self.current_numframe)
        
        # pixels to angle
        center_angle=3.14
        image_width_pixels=640
        center_pixel=int(image_width_pixels / 2)
        fov_radians=float(center_pixel/center_angle)
        
        
#        delta_ang=angle_radians-center_angle
#        angle_radians_int=int(delta_ang*100)
#        print 'angle_radians=',angle_radians_int
#        xuser=int(center_pixel+angle_radians_int)
        
        #print 'angle_radians=',angle_radians
        if angle_radians<0:
            delta_ang=angle_radians+center_angle
            delta_ang=int(delta_ang*120)
            xuser=center_pixel-delta_ang
        else:
            delta_ang=center_angle-angle_radians
            delta_ang=int(delta_ang*120)
            xuser=center_pixel+delta_ang

        
        #print 'xuser=',xuser
        
        cv2.rectangle(frame,(xuser,0),(xuser,480),(0,255,0),3)
        
        return frame
        
    def ExtractROI(self, event):
        

        
        frame = cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB)


        try:
            orig_height, orig_width = frame.shape[:2]
            y1 = self.roi_y.GetValue()
            y2 = y1 + self.roi_height.GetValue()
            y2 = min(y2, orig_height)
            x1 = self.roi_x.GetValue()
            x2 = x1 + self.roi_width.GetValue()
            x2 = min(x2, orig_width)

            frameRoi = frame[y1:y2, x1:x2]
            roi_width = x2-x1
            roi_height = y2-y1


            if hasattr(self, 'ROI_crosshair_pos') and self.enable_crosshairs:
                try:
                    cross_x = self.ROI_crosshair_pos[0]
                    cross_y = self.ROI_crosshair_pos[1]
                    frameRoi[0:roi_height, cross_x:cross_x+1] = [42,0,255]
                    frameRoi[cross_y:cross_y+1, 0:roi_width] = [42,0,255]

                    if hasattr(self, 'roi_click_down_pos'):
                        roi_x1 = self.roi_click_down_pos[0]
                        roi_y1 = self.roi_click_down_pos[1]

                        if cross_y>roi_y1:
                            frameRoi[0:roi_y1, 0:roi_width] = frameRoi[0:roi_y1, 0:roi_width]*.50
                            frameRoi[cross_y:roi_height, 0:roi_width] = frameRoi[cross_y:roi_height, 0:roi_width]*.50
                        else:
                            frameRoi[roi_y1:roi_height, 0:roi_width] = frameRoi[roi_y1:roi_height, 0:roi_width]*.50
                            frameRoi[0:cross_y, 0:roi_width] = frameRoi[0:cross_y, 0:roi_width]*.50

                        if cross_x>roi_x1:
                            frameRoi[0:roi_height, 0:roi_x1] = frameRoi[0:roi_height, 0:roi_x1]*.50
                            frameRoi[0:roi_height, cross_x:roi_width] = frameRoi[0:roi_height, cross_x:roi_width]*.50
                        else:
                            frameRoi[0:roi_height, roi_x1:roi_width] = frameRoi[0:roi_height, roi_x1:roi_width]*.50
                            frameRoi[0:roi_height, 0:cross_x] = frameRoi[0:roi_height, 0:cross_x]*.50


                except:
                    print 'couldn\'t draw crosshairs'
                    
                    traceback.print_exc()
            self.frameRoi = np.array(frameRoi , dtype = np.uint8)

            #frameRoi = cv2.cvtColor(self.frameRoi, cv2.COLOR_BGR2RGB)#GRAY2RGB)
            self.bmp2 = wx.BitmapFromBuffer(roi_width, roi_height, self.frameRoi)
            self.ImgControl2.SetBitmap(self.bmp2)

        except:
            traceback.print_exc()

        
        
if __name__ == '__main__':
    
    ms_face_call=rospy.ServiceProxy('/cognitivefaceapi/detect',Detect)
    
    
    rospy.init_node('users_analysis')
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--datapath", type=str, default='/media/robpin/063CAF573CAF4113/LCAS/collect_data_trajectories/data',
                        help="Data folder.  default='./data'.")
    parser.add_argument("--videofolders", type=str, default='head_camera_2017-01,head_camera_2017-02,head_camera_2017-03',
                        help="List of video folders separated by ',' in <datapath>/videos.  default='head_camera_2017-01,head_camera_2017-02'.")                        
    parser.add_argument("--mode", type=str, default='auto',
                        help="Analysis mode= manual")  
                    
    args = parser.parse_args()

    global path_folder_input
    global video_folders
    global path_videos
    
    path_folder_input=datapath+'/passive_interactions_manually'
    path_folder_output='/passive_interactions_uuids'
    path_videos=datapath+'/videos'
    
    defaul_folder_camera='head_camera_2017-01'
    default_video='head_camera_2017-01-04_06-00-01.avi'
    
    
    path_videos=args.datapath+'/videos'
    video_folders=args.videofolders
    try:
        video_folders=temp_video_folders.split(',')
    except:
        video_folders[0]=temp_video_folders
        
    mode=args.mode


    if mode =='manual':
        
        capture=cv2.VideoCapture(path_videos+'/'+default_folder_camera+'/'+default_video)
        app = wx.App()    
        window=ShowCapture(capture)
        window.Show()
        app.MainLoop()
        
    elif mode =='auto':
        ## MS face API
        mongo=MongoDb()
        mongo.auto_analysis()
  
        
    elif mode =='faceanalysis':
        ## MS face API
        mongo=MongoDb()
        mongo.getfaceuser()
        



    

