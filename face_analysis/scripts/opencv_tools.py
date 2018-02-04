#!/usr/bin/python


import rospy
import rospkg
import os

from std_msgs.msg import String
from trajectory_analysis_gui.srv import *
from ms_face_api.srv import *
from ms_face_api.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import glob
import os
import cv2

import json

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path 

path=rospack.get_path('users_analysis')
haarcascade_path=path+'/opencv_haarcascades'

class ExtractFrame():
           
    def __init__(self) :
        
        self.faceCascade = cv2.CascadeClassifier(haarcascade_path+"/haarcascade_face.xml")
        self.videoname=None
        self.videopath=None
        self.arrayvideofolders=None
        self.videofolder=None
        self.array_timestamps=[]
        self.capture=None
        self.videolength=None


        
    def load_info_video(self,path_videos,videofolder,name_file):
        
        self.videoname=name_file
        self.videopath=path_videos
        self.videofolder=videofolder
        
        with open(self.videopath+'/info_videos_'+self.videofolder+'.json', 'r') as f:
            videodata = json.load(f)
        #print 'videodata=',videodata
        for ivideo in videodata['video']:
            if ivideo['name']==self.videoname:
                
                self.array_timestamps=ivideo['timestamps']
                
                self.capture = cv2.VideoCapture(self.videopath+'/'+self.videofolder+'/'+self.videoname) #video_name is the video being called     
                while not self.capture.isOpened():
                    pass
                
                self.videolength = int(self.capture.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))

    def video_from_timestamp(self,timestamp):
        
        self.videopath=None
        for self.videofolder in self.arrayvideofolders:
            
            if self.videopath is None:
                with open(self.videopath+'/info_videos_'+self.videofolder+'.json', 'r') as f:
                    videodata = json.load(f)
                #print 'videodata=',videodata
                for ivideo in videodata['video']:
#                    print 'videoname=',ivideo['name']
#                    print 'inittimestamp=',ivideo['timestamps'][0]['timestamp']
#                    print 'endtimaestamp=',ivideo['timestamps'][len(ivideo['timestamps'])-1]['timestamp']
#                    print 'timestamp=',timestamp
                    if int(ivideo['timestamps'][0]['timestamp']) <= int(timestamp) <  int(ivideo['timestamps'][len(ivideo['timestamps'])-1]['timestamp']):
                        print 'IF'
                        self.array_timestamps=ivideo['timestamps']
                        self.videoname=ivideo['name']
                        self.videopath=self.videopath+'/'+self.videofolder+'/'+self.videoname
                        
                        self.array_timestamps=ivideo['timestamps']
                        
                        self.capture = cv2.VideoCapture(self.videopath+'/'+self.videofolder+'/'+self.videoname) #video_name is the video being called     
                        while not self.capture.isOpened():
                            pass
                        
                        self.videolength = int(self.capture.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
                        break
        print 'self.videopath=',self.videopath
        return self.videopath

    def show_frame(self,timestamp,name_window):
        
        numframe=self.estimate_frame(timestamp)
        print 'numframe=',numframe
        
        self.capture.set(1,numframe); # Where numframe is the frame you want
        ret, frame = self.capture.read() # Read the frame
        cv2.imshow(name_window, frame) # show frame on window
 
    def save_frame(self,numframe=None,timestamp=None):
        
        self.capture.set(1,numframe); # Where numframe is the frame you want
        ret, frame = self.capture.read() # Read the frame
        cv2.imshow('window_name', frame) # show frame on window
        key=cv2.waitKey(0)

    def show_gif(self,init_timestamp,end_timestamp,name_window):
        
        init_numframe=self.estimate_frame(init_timestamp)
        end_numframe=self.estimate_frame(end_timestamp)
        
        gif_numframe=init_numframe
        while gif_numframe<end_numframe:
                    
            self.capture.set(1,gif_numframe); # Where numframe is the frame you want
            ret, frame = self.capture.read() # Read the frame
            cv2.imshow(name_window, frame) # show frame on window
            
            time.sleep(0.2)
            #key=cv2.waitKey(0)
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
            gif_numframe +=1
            
        key=cv2.waitKey(0)
            
  
        
    def save_roi(self,numframe,rect_target,name,timestamp,path_folder_output):
   
        ymin=rect_target.y_offset
        ymax=rect_target.y_offset+rect_target.height
        xmin=rect_target.x_offset
        xmax=rect_target.x_offset+rect_target.width
        
        self.capture.set(1,numframe); # Where numframe is the frame you want
        ret, frame = self.capture.read() # Read the frame
        roi_new_user=frame[ymin:ymax,xmin:xmax]
        
        path_image=path_folder_output+'/'+name
        if not os.path.exists(path_image):
            os.makedirs(path_image)
            
        cv2.imwrite(path_image+'/'+str(timestamp)+'.png',roi_new_user)


        path_image=path_folder_output+'/frames'
        if not os.path.exists(path_image):
            os.makedirs(path_image)
            
        cv2.imwrite(path_image+'/'+str(timestamp)+'.png',frame)

        
    def rosimage(self,numframe=None,timestamp=None):
        
        resp=None
        
        if numframe is None:
            
            numframe=self.estimate_frame(timestamp)
            
        if numframe is not None:
        
            self.capture.set(1,numframe); # Where numframe is the frame you want
            ret, frame = self.capture.read() # Read the frame
            cv2.imshow('window_name', frame) # show frame on window
            
            #image=Image()
            bridge = CvBridge()
            ros_image=bridge.cv2_to_imgmsg(frame, "bgr8")
       
        return ros_image ,numframe
        
            
#    def ms_face_api_call_file(self,numframe=None,timestamp=None):
#        
#    #
#    #sensor_msgs/Image image
#    ## if a filename is given, load the file
#    #string filename
#    ## if a topic name is given, wait for an image on that topic
#    #string topic
#    #bool identify
#    #---
#    #ms_face_api/Faces faces
#        
#        self.capture.set(1,numframe); # Where numframe is the frame you want
#        ret, frame = self.capture.read() # Read the frame
#        cv2.imshow('window_name', frame) # show frame on window
#        filename='/media/robpin/data/videos/collect_data_y4/actively_engaged/'+str(timestamp)+'.png'
#        cv2.imwrite(filename,frame)
#        
#        image=Image()
#        
#        print 'CALLING SERVICE'
#    
#        #resp=ms_face_call(image,'/media/robpin/data/videos/collect_data_y4/2016_11/2016_11_11/843.png','',False)
#        resp=ms_face_call(image,filename,'',False)
#        
#        iface=0
#        for face in resp.faces.faces:
#            
#            color=(0, 255, 0)
#            cv2.rectangle(frame, (face.faceRectangle.x_offset, face.faceRectangle.y_offset), (face.faceRectangle.x_offset+face.faceRectangle.width, face.faceRectangle.y_offset+face.faceRectangle.height), color, 2)
#            font = cv2.FONT_HERSHEY_SIMPLEX
#            cv2.putText(frame,"Person "+str(iface),(face.faceRectangle.x_offset, face.faceRectangle.y_offset - 5), font, 0.7,color,2)
#            iface+=1
#        
#        cv2.imshow('window_name', frame) # show frame on window
#        
#        return resp 
        
    def estimate_frame(self, timestamp) :
        

        length_timestamp=None
        inf_frame=None
        length_frames=None
        inf_timestamp=None
        for i in range(len(self.array_timestamps)-1):
            if int(self.array_timestamps[i]['timestamp']) <= timestamp < int(self.array_timestamps[i+1]['timestamp']):
                
                length_timestamp=int(self.array_timestamps[i+1]['timestamp'])-int(self.array_timestamps[i]['timestamp'])
                length_frames=int(self.array_timestamps[i+1]['num_frame'])-int(self.array_timestamps[i]['num_frame'])
                inf_frame=int(self.array_timestamps[i]['num_frame'])
                inf_timestamp=int(self.array_timestamps[i]['timestamp'])
                
        
        
        nframe=None        
        if length_frames is not None:
          
            video_rate=float(length_timestamp)/float(length_frames)

            nframe=float(float(timestamp-inf_timestamp)/video_rate)+inf_frame
            
            nframe=int(round(nframe))
  
        
        return nframe
    
    def estimate_timestamp(self, numframe) :
        
             
        #print 'current_timestamp=',timestamp
        
        length_timestamp=None
        inf_frame=None
        length_frames=None
        inf_timestamp=None
        for i in range(len(self.array_timestamps)-1):
            if int(self.array_timestamps[i]['num_frame']) <= numframe < int(self.array_timestamps[i+1]['num_frame']):
                
                length_timestamp=int(self.array_timestamps[i+1]['timestamp'])-int(self.array_timestamps[i]['timestamp'])
                length_frames=int(self.array_timestamps[i+1]['num_frame'])-int(self.array_timestamps[i]['num_frame'])
                inf_frame=int(self.array_timestamps[i]['num_frame'])
                inf_timestamp=int(self.array_timestamps[i]['timestamp'])
                
        #print 'length_frames=',length_frames
        #print 'inf_frame=',inf_frame
        
        
        nframe=None        
        if length_frames is not None:
          
            video_rate=float(length_timestamp)/float(length_frames)

#            nframe=float(float(timestamp-inf_timestamp)/video_rate)+inf_frame
#            
#            nframe=int(round(nframe))


            timestamp=float(float(numframe-inf_frame)*video_rate)+inf_timestamp
            
            timestamp=int(round(timestamp))
        #print 'nframe=',nframe
        
        
        return timestamp
        
    def detect_faces(self ,timestamp,framesnumber=1,increment=1):

        faces=None
        

            
        numframe=self.estimate_frame(timestamp)    
        newframe=numframe
        
        if numframe is not None:
            
            indexframe=0
            countframesnumber=0
            while countframesnumber < framesnumber:
            
                
                try:
                    newframe=numframe+indexframe
                    self.capture.set(1,newframe); # Where numframe is the frame you want
                    ret, frame = self.capture.read() # Read the frame
                    
                    
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    cv2.imshow('face detection', gray)
                    # Detect faces in the image
                    faces = self.faceCascade.detectMultiScale(
                        gray,
                        scaleFactor=1.1,
                        minNeighbors=5,
                        minSize=(30, 30),
                        flags = cv2.cv.CV_HAAR_SCALE_IMAGE
                    )
                    
                    
                    indexframe=indexframe+increment
                    countframesnumber+=1

                    
                    
                    if len(faces)>0:
                        print 'detect_faces break'
                        break
                except:
                    pass
                    

        return faces,newframe
        
    def detect_faces_image(self ,image):

        faces=None
        frame=cv2.imread(image)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect faces in the image
        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags = cv2.cv.CV_HAAR_SCALE_IMAGE
        )
                    

        return faces
