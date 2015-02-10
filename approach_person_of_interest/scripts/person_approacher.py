#! /usr/bin/env python

import rospy
from actionlib import SimpleActionServer, SimpleActionClient
from approach_person_of_interest.srv import Activate 


    
class PersonApproacher(object):

    def __init__(self):
        self.active=False
        self.activate=rospy.Service('info_terminal/activate_approacher',Activate,self.activate_cb)

        #put stuff

    def activate_cb(self,req): 
        self.active=req.activate
        return True
    

    def main(self):
        # Wait for control-c
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('person_approacher')
    person_app = PersonApproacher()
    person_app.main()
