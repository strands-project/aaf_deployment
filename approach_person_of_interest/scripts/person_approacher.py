#! /usr/bin/env python

import math
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, Pose

from actionlib import SimpleActionServer, SimpleActionClient
from approach_person_of_interest.srv import Activate 

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import DemandTask, SetExecutionStatus

    
class PersonApproacher(object):

    def __init__(self):
        self.active=True
        self.activate=rospy.Service('info_terminal/activate_approacher',Activate,self.activate_cb)
        self.card_seer=rospy.Subscriber("/socialCardReader/commands", String, self.card_callback)
        self.card_pose_getter=rospy.Subscriber("/socialCardReader/cardposition", PoseStamped, self.card_pose_callback)
        self.tf_listener=tf.TransformListener()
        self.card_pose=None
        

    def activate_cb(self,req): 
        self.active=req.activate
        return True

    def card_callback(self, data):
        rospy.loginfo("Saw a card")
        if data.data=='INFO_TERMINAL':
            if self.active:
                rospy.loginfo("I'm moving towards a Card pose in %s", self.card_pose)
                xtion_frame_pose=PointStamped(point=self.card_pose.position)
                xtion_frame_pose.header.frame_id='/head_xtion_rgb_optical_frame'
                map_frame_point=self.tf_listener.transformPoint('map', xtion_frame_pose)

                goal_pose=PoseStamped()
                goal_pose.header.frame_id='/map'
                goal_pose.pose.position.x=map_frame_point.point.x
                goal_pose.pose.position.y=map_frame_point.point.y
                
                current_robot_pose = rospy.wait_for_message("/robot_pose",Pose)
                dist = math.sqrt( (current_robot_pose.position.x - goal_pose.pose.position.x )**2 + 
                                (current_robot_pose.position.y - goal_pose.pose.position.y )**2)
                goal_pose.pose.position.x = (goal_pose.pose.position.x - current_robot_pose.position.x) / dist * (dist - 0.6)
                goal_pose.pose.position.y = (goal_pose.pose.position.y - current_robot_pose.position.y) / dist * (dist - 0.6)
                
                dx=goal_pose.pose.position.x - map_frame_point.point.x
                dy=goal_pose.pose.position.y - map_frame_point.point.y
                th=math.atan2(dy,dx)
                orientation=tf.transformations.quaternion_from_euler(0,0,th)
                goal_pose.pose.orientation.x=orientation[0]
                goal_pose.pose.orientation.y=orientation[1]
                goal_pose.pose.orientation.z=orientation[2]
                goal_pose.pose.orientation.w=orientation[3]
                rospy.loginfo("going to pose: %s ", goal_pose) 
            else:
                rospy.loginfo("I saw a info request but I'm pretty occupied")
                #say sorry mate
    
    def card_pose_callback(self, data):
        self.card_pose=data.pose

        
        
        

    def main(self):
        # Wait for control-c
        rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node('person_approacher')
    person_app = PersonApproacher()
    person_app.main()
