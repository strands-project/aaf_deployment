#! /usr/bin/env python
import rospy
import actionlib
import tf

from aaf_walking_group.msg import GuidingAction, GuidingServer
import topological_navigation.msg
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msgs import Odometry


class GuidingServer:
  def __init__(self):
      self.server = actionlib.SimpleActionServer('guiding', GuidingAction, self.execute, False)
      self.server.start()
      self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
      self.card_subscriber =rospy.Subscriber("card", String, self.callback)
      self.odom_subscriber =rospy.Subscriber("odom", Odometry, self.callback)
      self.last_location = Odometry()
      self.pause = 0;
      self.pause_service = rospy.ServiceProxy('/monitored_navigation/pause_nav', Bool)
      self.tf.listener = tf.TransformListener()
      
  def execute(self, goal):
      #call send keypoint (topological navigation)
  
      try:
          (trans,rot) = listener.lookupTransform('/base_link', '/map', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
          
      self.last_location.pose.pose.x = trands[0]
      self.last_location.pose.pose.y = trans[1]
      self.client.wait_for_server()
      navgoal = topological_navigation.msg.GotoNodeGoal()
      navgoal.target = targ
      self.client.send_goal(navgoal)
      self.client.wait_for_result()
      ps = self.client.get_result() 
      print ps
      
      self.server.set_succeeded()
      #what to do when it fails?
      
   def _on_node_shutdown(self):
       self.client.cancel_all_goals()
      
   def card_callback(self, data):
       if self.pause == 1
           #call action server
           #...
           try:
               self.pause_service(0)
               self.pause = 0
           except rospy.ServiceException, e:
               print "Service call failed: %s" % e
       

   def odom_callback(self, data):
       x = data.pose.pose.x - last_location.pose.pose.x
       y = data.pose.pose.y - last_location.pose.pose.y
       
       lenght = sqrt(x*x + y*y)
       
       if lengh => 2.0:
           try:
            self.pause_service(1)
            self.pause = 1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
       
       
if __name__ == '__main__':
  rospy.init_node('guiding_server')
  server = GuidingServer()
  
  rospy.spin()