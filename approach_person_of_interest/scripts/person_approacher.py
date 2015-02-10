#! /usr/bin/env python

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped

from actionlib import SimpleActionServer, SimpleActionClient
from approach_person_of_interest.srv import Activate 


    
class PersonApproacher(object):

    def __init__(self):
        self.active=False
        self.activate=rospy.Service('info_terminal/activate_approacher',Activate,self.activate_cb)
        self.card_seer=rospy.Subscriber("/socialCardReader/commands", String, self.card_callback)
        self.card_pose_getter=rospy.Subscriber("/socialCardReader/cardposition", PoseStamped, self.card_pose_callback)
        self.tf_ros = tf.TransformerROS()

    def activate_cb(self,req): 
        self.active=req.activate
        return True

    def card_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I see a  %s", data.data)
    
    def card_pose_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The Card pose is %s", data.pose)
        xtion_frame_pose=PointStamped(point=data.pose.position)
        xtion_frame_pose.header.frame_id='/head_xtion_rgb_optical_frame'
        map_frame_pose=self.tf_ros.transformPoint('base_link', xtion_frame_pose)
        
        print "TESTE", map_frame_pose
        

    def main(self):
        # Wait for control-c
        rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node('person_approacher')
    person_app = PersonApproacher()
    person_app.main()