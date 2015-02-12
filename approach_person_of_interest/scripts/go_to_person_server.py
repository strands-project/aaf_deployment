#! /usr/bin/env python

import roslib
import rospy
import actionlib
#import approach_person_of_interest
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from approach_person_of_interest.msg import *
from monitored_navigation import *
from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationGoal
import strands_webserver.client_utils

class goToPersonAction(object):
  _feedback = goToPersonFeedback()
  _result   = goToPersonResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, approach_person_of_interest.msg.GoToPersonAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    rospy.loginfo("Action server up: %s"%self._action_name)
    self._mon_nav_client = actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)    
    print 'Waiting for monitored navigation to start'
    self._mon_nav_client.wait_for_server();
    print 'Monitored navigation is ready'

  def execute_cb(self, goal):
    # helper variables
    print goal.go_to_person
    if goal.go_to_person:
	print 'going to person'
	mon_nav_goal=MonitoredNavigationGoal(action_server='move_base', target_pose=goal.pose)
        self._mon_nav_client.send_goal(mon_nav_goal)

    strands_webserver.client_utils.display_url(0, 'http://localhost:8080')
   
    #while
#	rospy.sleep(1)
	
    success = True

    if success:
      self._result.success = True
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)

  def send_feedback(self, txt):
	self._feedback.status = txt
	self._as.publish_feedback(self._feedback)
	rospy.loginfo(txt)

if __name__ == '__main__':
  rospy.init_node('go_to_person_action')
  goToPersonAction(rospy.get_name())
  rospy.spin()
