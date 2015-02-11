#! /usr/bin/env python
import rospy
import actionlib
import tf
import numpy

from aaf_walking_group.msg import GuidingAction
from aaf_walking_group.msg import EmptyAction, EmptyActionGoal
import topological_navigation.msg
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


class GuidingServer():
    def __init__(self):
        self.server = actionlib.SimpleActionServer('guiding', GuidingAction, self.execute, False)
        self.server.start()
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        self.empty_client = actionlib.SimpleActionClient('wait_for_participant', EmptyAction)
        self.card_subscriber =rospy.Subscriber("/socialCardReader/QSR_generator", String, self.card_callback)
        self.odom_subscriber =rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.last_location = Odometry()
        self.pause = 0;
        self.pause_service = rospy.ServiceProxy('/monitored_navigation/pause_nav', Bool)
        self.tf.listener = tf.TransformListener()
      
    def execute(self, goal):
        #call send keypoint (topological navigation)
      
        try:
            (trans,rot) = tf.listener.lookupTransform('/base_link', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
          
        self.last_location.pose.pose.x = trans[0]
        self.last_location.pose.pose.y = trans[1]
        self.client.wait_for_server()
        navgoal = topological_navigation.msg.GotoNodeGoal()
        navgoal.target = goal.waypoint
        self.client.send_goal(navgoal)
        self.client.wait_for_result()
        ps = self.client.get_result() 
        print ps
        self.server.set_succeeded()
        #what to do when it fails?
    
    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
      
    def card_callback(self, data):
        if self.pause == 1:
            #call action server
            if data.data == 'near':
                self.empty_client.send_goal_and_wait(EmptyActionGoal())
            try:
                self.pause_service(0)
                self.pause = 0
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
       

    def odom_callback(self, data):
        x = data.pose.pose.position.x - self.last_location.pose.pose.position.x
        y = data.pose.pose.position.y - self.last_location.pose.pose.position.y
           
        lenght = numpy.sqrt(x*x + y*y)
           
        if lenght >= 2.0:
            try:
                self.pause_service(1)
                self.pause = 1
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
       
       
if __name__ == '__main__':
  rospy.init_node('guiding_server')
  server = GuidingServer()
  
  rospy.spin()