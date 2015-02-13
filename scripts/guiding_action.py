#! /usr/bin/env python
import rospy
import actionlib
import numpy
import math

from aaf_walking_group.msg import GuidingAction, GuidingGoal
from aaf_walking_group.msg import EmptyAction, EmptyActionGoal
import topological_navigation.msg
from std_msgs.msg import String
from strands_navigation_msgs.srv import PauseResumeNav
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path


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
        self.card_subscriber = rospy.Subscriber(
            "/socialCardReader/QSR_generator",
            String,
            self.card_callback
        )
        self.odom_subscriber = rospy.Subscriber(
            "odom",
            Odometry,
            self.odom_callback,
            queue_size=1
        )
        
        self.path_subscriber = rospy.Subscriber(
            "/move_base/DWAPlannerROS/local_plan",
            Path,
            self.path_callback,
            queue_size = 1
        )
        
        self.last_location = Odometry()
        self.pause = 0
        self.begin = 1
        self.counter = 0

        self.client_walking_interface = actionlib.SimpleActionClient(
            'walking_interface_server',
            GuidingAction
        )
        self._client.wait_for_server()

    def execute(self, goal):
        self.begin = 1
        navgoal = topological_navigation.msg.GotoNodeGoal()
        navgoal.target = goal.waypoint
        self.client.send_goal(navgoal)

        self.client.wait_for_result()
        ps = self.client.get_result()
        print ps
        self.server.set_succeeded()

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()

    def card_callback(self, data):
        if self.pause == 1:
            # call action server
            if data.data == 'near':
                self.odom_subscriber = None
                self.empty_client.send_goal_and_wait(EmptyActionGoal())
                try:
                    pause_service = rospy.ServiceProxy(
                        '/monitored_navigation/pause_nav',
                        PauseResumeNav
                    )
                    pause_service(0)
                    self.pause = 0
                    print "the guy is near, fear him"
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                self.odom_subscriber = rospy.Subscriber("odom", Odometry,
                                                        self.odom_callback)

    def odom_callback(self, data):
        if self.begin == 0:
            if self.counter == 10:
                x = data.pose.pose.position.x - \
                    self.last_location.pose.pose.position.x
                y = data.pose.pose.position.y - \
                    self.last_location.pose.pose.position.y

                lenght = numpy.sqrt(x*x + y*y)

                if lenght >= 2.0:
                    # self.odom_subscriber.unregister()
                    print "reached 2.0 meters"
                    try:
                        pause_service = rospy.ServiceProxy(
                            '/monitored_navigation/pause_nav',
                            PauseResumeNav
                        )
                        pause_service(1)
                        self.pause = 1
                        self.begin = 1
                        print "pause"
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                    # self.odom_subscriber = rospy.Subscriber("odom", Odometry, self.odom_callback)
                self.counter = 0
        else:
            self.last_location.pose.pose.position.x = data.pose.pose.position.x
            self.last_location.pose.pose.position.y = data.pose.pose.position.y
            self.begin = 0

        self.counter += 1
        
    def path_callback(self, path): 
        yDiff = path.poses[-1].pose.position.y - path.poses[0].pose.position.y
        xDiff = path.poses[-1].pose.position.x - path.poses[0].pose.position.x
        
        angle = math.degrees(math.atan2(yDiff,xDiff))
        
        if angle > -90 and angle < 85:
            direction = 'right'
            walking_interface_goal = GuidingGoal()
            walking_interface_goal.waypoint = direction
            self.client_walking_interface.send_goal_and_wait(walking_interface_goal)
            self.client_walking_interface.get_result()
        elif angle >= 85 and angle <= 95:
            direction = 'straight'
            walking_interface_goal = GuidingGoal()
            walking_interface_goal.waypoint = direction
            self.client_walking_interface.send_goal_and_wait(walking_interface_goal)
            self.client_walking_interface.get_result()
        else:
            direction = 'left'
            walking_interface_goal = GuidingGoal()
            walking_interface_goal.waypoint = direction
            self.client_walking_interface.send_goal_and_wait(walking_interface_goal)
            self.client_walking_interface.get_result()
        

if __name__ == '__main__':
    rospy.init_node('guiding_server')
    server = GuidingServer()
    rospy.spin()
