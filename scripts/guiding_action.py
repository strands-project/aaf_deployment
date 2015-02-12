#! /usr/bin/env python
import rospy
import actionlib
#import tf
import numpy
import mutex

from aaf_walking_group.msg import GuidingAction, GuidingGoal
from aaf_walking_group.msg import EmptyAction, EmptyActionGoal
import topological_navigation.msg
from std_msgs.msg import String
from strands_navigation_msgs.srv import PauseResumeNav
from nav_msgs.msg import Odometry


class GuidingServer():

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'guiding',
            GuidingAction,
            self.execute,
            False
        )
        self.server.start()
        self.client = actionlib.SimpleActionClient(
            'topological_navigation',
            topological_navigation.msg.GotoNodeAction
        )
        self.client.wait_for_server()
        self.empty_client = actionlib.SimpleActionClient(
            'wait_for_participant',
            EmptyAction
        )
        self.empty_client.wait_for_server()
<<<<<<< HEAD
        self.card_subscriber = rospy.Subscriber(
            "/socialCardReader/QSR_generator",
            String,
            self.card_callback
        )
        self.odom_subscriber = rospy.Subscriber(
            "odom",
            Odometry,
            self.odom_callback
        )
        self.last_location = Odometry()
        self.pause = 0
        self.begin = 0

        self.client_walking_interface = actionlib.SimpleActionClient(
            'walking_interface_server',
            GuidingAction
        )
        self._client.wait_for_server()

    def execute(self, goal):
        self.begin = 1

=======
        self.card_subscriber =rospy.Subscriber("/socialCardReader/QSR_generator", String, self.card_callback)
        self.odom_subscriber =rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1 )
        self.last_location = Odometry()
        self.pause = 0;
        self.begin = 1;
        self.counter = 0;
      
    def execute(self, goal):
      
        self.begin = 1;
>>>>>>> 963e31b45d6832cd48835badbe5937bfb4e91984
        navgoal = topological_navigation.msg.GotoNodeGoal()
        navgoal.target = goal.waypoint
        self.client.send_goal(navgoal)

        # calling walking interface server
        direction = 'right'
        walking_interface_goal = GuidingGoal()
        walking_interface_goal.waypoint = direction
        self.client_walking_interface.send_goal_and_wait(walking_interface_goal)
        self.client_walking_interface.get_result()

        # self.client.wait_for_result()
        ps = self.client.get_result()
        self.server.set_succeeded()
        print ps
        # what to do when it fails?

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()

    def card_callback(self, data):
        if self.pause == 1:
            # call action server
            if data.data == 'near':
       		self.odom_subscriber = None
                self.empty_client.send_goal_and_wait(EmptyActionGoal())
<<<<<<< HEAD
            try:
                pause_service = rospy.ServiceProxy(
                    '/monitored_navigation/pause_nav',
                    PauseResumeNav
                )
                pause_service(0)
                self.pause = 0
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

    def odom_callback(self, data):
        if self.begin == 0:
            x = data.pose.pose.position.x - self.last_location.pose.pose.position.x
            y = data.pose.pose.position.y - self.last_location.pose.pose.position.y

            lenght = numpy.sqrt(x*x + y*y)

            if lenght >= 2.0:
                try:
                    pause_service = rospy.ServiceProxy(
                        '/monitored_navigation/pause_nav',
                        PauseResumeNav
                    )
                    pause_service(1)
                    self.pause = 1
                    self.begin = 1
=======
                try:
                    pause_service = rospy.ServiceProxy('/monitored_navigation/pause_nav', PauseResumeNav)
                    pause_service(0)
                    self.pause = 0
                    print "the guy is near, fear him"
>>>>>>> 963e31b45d6832cd48835badbe5937bfb4e91984
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
        	self.odom_subscriber =rospy.Subscriber("odom", Odometry, self.odom_callback)


    def odom_callback(self, data):
        if self.begin == 0:
            if self.counter == 10:
                x = data.pose.pose.position.x - self.last_location.pose.pose.position.x
                y = data.pose.pose.position.y - self.last_location.pose.pose.position.y
                   
                lenght = numpy.sqrt(x*x + y*y)
                   
                if lenght >= 2.0:
                   # self.odom_subscriber.unregister()
                    print "reached 2.0 meters"
                    try:
                        pause_service = rospy.ServiceProxy('/monitored_navigation/pause_nav', PauseResumeNav)
                        pause_service(1)
                        self.pause = 1
                        self.begin = 1
                        print "pause"
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                    #self.odom_subscriber =rospy.Subscriber("odom", Odometry, self.odom_callback)
                self.counter = 0
        else:
<<<<<<< HEAD
            self.last_location.pose.pose.position.x = data.pose.pose.position.x
            self.last_location.pose.pose.position.y = data.pose.pose.position.y
            self.begin = 0


=======
            self.last_location.pose.pose.position.x = data.pose.pose.position.x;
            self.last_location.pose.pose.position.y = data.pose.pose.position.y;
            self.begin = 0;
        
        self.counter += 1
            
                        
       
       
>>>>>>> 963e31b45d6832cd48835badbe5937bfb4e91984
if __name__ == '__main__':
    rospy.init_node('guiding_server')
    server = GuidingServer()
    rospy.spin()
