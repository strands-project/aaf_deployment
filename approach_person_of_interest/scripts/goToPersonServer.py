#! /usr/bin/env python

import roslib
import rospy
import actionlib
import approach_person_of_interest
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped

class goToPersonAction(object):
  _feedback = approach_person_of_interest.msg.goToPersonFeedback()
  _result   = approach_person_of_interest.msg.goToPersonResult()

  def __init__(self, name):
#! /usr/bin/env python    
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, approach_person_of_interest.msg.goToPersonAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True

    if success:
      self._result.success = true
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)

if __name__ == '__main__':
  rospy.init_node('go_to_person_action')
  goToPersonAction(rospy.get_name())
  rospy.spin()
