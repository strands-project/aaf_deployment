#! /usr/bin/env python

import rospy
from std_msgs.msg import String

from actionlib import SimpleActionServer, SimpleActionClient
from approach_person_of_interest.srv import Activate 


    
class PersonApproacher(object):

    def __init__(self):
        self.active=False
        self.activate=rospy.Service('info_terminal/activate_approacher',Activate,self.activate_cb)
        self.card_seer=rospy.Subscriber("/socialCardReader/commands", String, self.card_callback)
        #put stuff

    def activate_cb(self,req): 
        self.active=req.activate
        return True

    def card_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    

    def main(self):
        # Wait for control-c
        rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node('person_approacher')
    person_app = PersonApproacher()
    person_app.main()