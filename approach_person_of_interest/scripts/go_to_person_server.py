#! /usr/bin/env python
import roslib
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, PointStamped
from approach_person_of_interest.msg import *


from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32, Bool, Int32
from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationGoal
import strands_webserver.client_utils
from strands_executive_msgs.srv import IsTaskInterruptible

import strands_gazing.msg
import scitos_ptu.msg
from monitored_navigation import *
from flir_pantilt_d46.msg import *

class GoToPersonAction(object):
  _feedback = GoToPersonFeedback()
  _result   = GoToPersonResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, approach_person_of_interest.msg.GoToPersonAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    rospy.Service(self._action_name + '_is_interruptible', IsTaskInterruptible, self.is_interruptible)
    
    # this will be set to true while actively engaging with someone
    self._is_in_active_time = False    
    self._activity_timer = None

    rospy.loginfo("Action server up: %s"%self._action_name)
    self._mon_nav_client = actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)    
    print 'Waiting for monitored navigation to start'
    self._mon_nav_client.wait_for_server();
    print 'Monitored navigation is ready'
    self._timeout = 100
    rospy.Subscriber("info_terminal/active_screen", Int32, self.button_pressed_callback)
    print 'Waiting for ptu...'
    self.ptuclient = actionlib.SimpleActionClient("SetPTUState", flir_pantilt_d46.msg.PtuGotoAction)
    self.ptuclient.wait_for_server()
    print 'ptu ready!'
    self.ptugoal = flir_pantilt_d46.msg.PtuGotoGoal()
    self.ptugoal.pan_vel = 60
    self.ptugoal.tilt_vel = 60
    
    # blink eyes when there is an interaction (i.e. GUI button pressed)
    self.pub = rospy.Publisher('/head/commanded_state', JointState, queue_size=2)

    # Publishing the gaze pose
    self.gaze_topic_pub=rospy.Publisher('/info_terminal/gaze_pose',PoseStamped, queue_size=2)
    # Create a gaze action client
    self.gaze_act_client = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)

  def is_interruptible(self, req):
    return not self._is_in_active_time

  def _reset_activitity_timer(self, event):
    rospy.loginfo('Activity timer complete')
    self._activity_timer = None
    self._is_in_active_time = False

  def _start_activity_timer(self):
    if self._activity_timer is None:
      rospy.loginfo('Starting activity timer')
      self._is_in_active_time = True
      self._activity_timer = rospy.Timer(rospy.Duration(self._timeout), self._reset_activitity_timer, oneshot=True)
    else:
      rospy.loginfo('Activity timer to be extended')
      # shutdown previous timer and try again
      self._activity_timer.shutdown()
      self._activity_timer = None
      self._start_activity_timer()

  def execute_cb(self, goal):
    # helper variables
    print goal.go_to_person

    # goal.timeout is how long to run this behaviour for
    # self._timeout is how long to extend behaviour for on 
    self._timeout_after = rospy.get_rostime() + rospy.Duration(goal.timeout)



    if goal.go_to_person:
        print 'going to person'

        # prevent interruption
        self._is_in_active_time = True
        self.send_feedback('going to person')
        mon_nav_goal=MonitoredNavigationGoal(action_server='move_base', target_pose=goal.pose)
        self._mon_nav_client.send_goal(mon_nav_goal)
        self._mon_nav_client.wait_for_result()
        self.send_feedback('Reached the right position')
        # Send goal to gaze action server
        gaze_dir_goal=setPose(action_server='gaze_at_pose', topic_name='/info_terminal/gaze_pose')
        self.gaze_act_client.send_goal(gaze_dir_goal)
        self.gaze_pose.publish(goal.pose)

        # assume some default activity
        self._start_activity_timer()

    strands_webserver.client_utils.display_url(0, 'http://localhost:8080')
    self.currentPan=-180
    self.currentTilt=0
    self.head_command = JointState() 
    self.head_command.name=["HeadPan", "HeadTilt"] 
    self.head_command.position=[self.currentPan, self.currentTilt]
    self.pub.publish(self.head_command)


    self.send_feedback('Turning the camera to the person...')
    #turn head cam to person
    self.ptugoal.pan = 160
    self.ptugoal.tilt = 20
    self.ptuclient.send_goal(self.ptugoal)
    self.ptuclient.wait_for_result()
    self.send_feedback('camera turned successfully!')


    rate = rospy.Rate(1)
    # preempt will not be requested while activity is happening 
    while not rospy.is_shutdown() and not self._as.is_preempt_requested() and rospy.get_rostime() < self._timeout_after:
        # loop for duration 
        rate.sleep()
    
    self.exit_as()


  

  def button_pressed_callback(self, active_screen):
      # reset timeout

      rospy.loginfo('button_pressed_callback')

      self._start_activity_timer()
   
      #blink eyes
      self.eyelid_command = JointState()
      self.eyelid_command.name=["EyeLids"]
      self.eyelid_command.position=[20]
      self.pub.publish(self.eyelid_command)
      rospy.sleep(0.5)
      self.eyelid_command.position=[100]
      self.pub.publish(self.eyelid_command)

#just for testing the ptu
      self.ptugoal.pan = 160
      self.ptugoal.tilt = 20
      self.ptuclient.send_goal(self.ptugoal)
      self.ptuclient.wait_for_result()


      self.send_feedback('Turning the camera to the person...')
      #turn head cam to person
      self.ptugoal.pan = 160
      self.ptugoal.tilt = 20
      self.ptuclient.send_goal(self.ptugoal)
      self.ptuclient.wait_for_result()
      self.send_feedback('camera turned successfully!')

      self.exit_as()


#      self.currentPan=20
#      self.currentTilt=0
#      self.head_command = JointState() 
#      self.head_command.name=["HeadPan", "HeadTilt"] 
#      self.head_command.position=[self.currentPan, self.currentTilt]
#      self.pub.publish(self.head_command)

  def send_feedback(self, txt):
    self._feedback.status = txt
    self._as.publish_feedback(self._feedback)
    rospy.loginfo(txt)

  def exit_as(self):
    self.send_feedback('Turning head camera to default position...')
    #turn head cam to person
    self.ptugoal.pan = 0
    self.ptugoal.tilt = 0
    self.ptuclient.send_goal(self.ptugoal)
    self.ptuclient.wait_for_result()
    self.send_feedback('head camera turned successfully to default position!')
    self._result.success = True
    rospy.loginfo('%s: Succeeded' % self._action_name)
    self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('go_to_person_action')
  GoToPersonAction(rospy.get_name())
  rospy.spin()

# Add stuff to move head randomly.
# Add stuff to rotate head and blink eyes at target Point
# Add stuff to check time out twice and then warn and move away
# Add stuff to rotate head also by 180deg after robot rotate
