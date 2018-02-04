import glob, os
import argparse


from pymongo import MongoClient
import json
import yaml
from bson import json_util

import pprint

from datetime import timedelta, datetime
import time
import cv
import cv2
#import pytz

import rospy

import wx
    
#collection='people_perception'    

offset_hour=-1
offset_secs=5


def ask(parent=None, message='', default_value=''):
    dlg = wx.TextEntryDialog(parent, message, defaultValue=default_value)
    dlg.ShowModal()
    result = dlg.GetValue()
    dlg.Destroy()
    return result
    
class init_timestamp_video(object):

    def __init__(self,path_videos,video_folders) :
        

        
        for folder in video_folders:
            
            video_array=[]
            
            os.chdir(path_videos+"/"+folder)
            for file in sorted(glob.glob("*.avi")):
                
                print(file)

                temp_name=file.split('_')
                date=temp_name[2].split('-')
                tmp=temp_name[3].split('.')
                time=tmp[0].split('-')
                
                date=datetime(int(date[0]), int(date[1]), int(date[2]),int(time[0]) , int(time[1]), int(time[2]))
                date=date+ timedelta(hours=-1)
                date=date+ timedelta(seconds=5)
                
                init_timestamp=int(date.strftime("%s"))
                print 'timestamp=',init_timestamp
                start_time=date.strftime("%H:%M:%S")
              
                array_timestamp=self.ask_timestamp(file)
          
                video_info={"name":file,"timestamps":array_timestamp,"start_time":start_time}
            
                video_array.append(video_info)
                
            data_videos={"video":video_array}        
            jsonData=json.dumps(data_videos,indent=2 )
            #print 'jsonData=',jsonData
            
            os.chdir("./")
            fh = open('info_videos_'+folder+'.json',"w")
            fh.write(jsonData)
            fh.close()
            print 'SAVED'
                

    def ask_timestamp(self,videofile):
        cap = cv2.VideoCapture(videofile) #video_name is the video being called
        frame_no=0
        percent_video=[0.0,0.25,0.50,0.75,1.0]
        timestamp_array=[]
        while not cap.isOpened():
            pass
        
        length = int(cap.get(cv.CV_CAP_PROP_FRAME_COUNT))
        app = wx.App()
        app.MainLoop()
        
        for current_percent in percent_video:
            
            
            num_frame=int(length*current_percent)
            if num_frame > length-1:
                num_frame=length-1
            cap.set(1,num_frame); # Where frame_no is the frame you want
            ret, frame = cap.read() # Read the frame
            
    
            cv2.imshow("TIMESTAMP", frame)
            
            
    
            # Call Dialog
            entry_timestamp = ask(message = 'What is the timestamp for this image?')
    
            cv2.destroyAllWindows()        
            current_timestamp={"num_frame":num_frame,"timestamp":entry_timestamp}
            timestamp_array.append(current_timestamp)
            
        return timestamp_array

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--videospath", type=str, default='./data/videos',
                        help="Videos folder.  default='./data/videos'.")
    parser.add_argument("--videofolders", type=str, default='head_camera_2017-01,head_camera_2017-02,head_camera_2017-03',
                        help="List of video folders separated by ',' in <datapath>/videos.  default='head_camera_2017-01,head_camera_2017-02'.")                        
 
    args = parser.parse_args()
    


    path_videos=args.videospath
    temp_video_folders=args.videofolders
    try:
        path_video_folders=temp_video_folders.split(',')
    except:
        path_video_folders[0]=temp_video_folders
        
    server = init_timestamp_video(path_videos,path_video_folders)
    
