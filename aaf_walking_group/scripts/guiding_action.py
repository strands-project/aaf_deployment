#! /usr/bin/env python
import rospy
import actionlib
import numpy

from aaf_walking_group.msg import GuidingAction
from aaf_walking_group.msg import EmptyAction, EmptyActionGoal
import topological_navigation.msg
from std_msgs.msg import String
from strands_navigation_msgs.srv import PauseResumeNav
from sound_player_server.srv import PlaySoundService
from nav_msgs.msg import Odometry


class GuidingServer():

    def __init__(self, name):
        rospy.loginfo("Creating "+name+" server")
        self.server = actionlib.SimpleActionServer(
            '/guiding',
            GuidingAction,
            self.execute,
            False
        )
        self.server.register_preempt_callback(self.preempt_callback)

        rospy.loginfo("Creating topo nav client...")
        self.client = actionlib.SimpleActionClient(
            '/topological_navigation',
            topological_navigation.msg.GotoNodeAction
        )
        self.client.wait_for_server()
        rospy.loginfo(" ... done ")
        rospy.loginfo("Creating wait client...")
        self.empty_client = actionlib.SimpleActionClient(
            '/wait_for_participant',
            EmptyAction
        )
        self.empty_client.wait_for_server()
        rospy.loginfo(" ... done ")
        rospy.loginfo("Creating interface client...")
        self.client_walking_interface = actionlib.SimpleActionClient(
            '/walking_interface_server',
            EmptyAction
        )
        self.client_walking_interface.wait_for_server()
        rospy.loginfo(" ... done ")
        self.card_subscriber = rospy.Subscriber(
            "/socialCardReader/QSR_generator",
            String,
            self.card_callback
        )
        self.odom_subscriber = rospy.Subscriber(
            "/odom",
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        self.last_location = Odometry()
        self.pause = 0
        self.begin = 0
        self.counter = 0
        self.distance = 5.0

        rospy.loginfo(" ... starting "+name)
        self.server.start()
        rospy.loginfo(" ... started "+name)

    def execute(self, goal):
        self.begin = 0
        self.pause = 0
        self.distance = goal.distance
        self.client_walking_interface.send_goal(EmptyActionGoal())
        navgoal = topological_navigation.msg.GotoNodeGoal()
        navgoal.target = goal.waypoint
        self.client.send_goal(navgoal)

        self.client.wait_for_result()
        ps = self.client.get_result()
        print "Guiding result:", ps
        self.client_walking_interface.cancel_goal()
        if not self.server.is_preempt_requested():
            self.server.set_succeeded()
        else:
            self.server.set_preempted()


    def preempt_callback(self):
        rospy.logwarn("Guiding action preempt requested")
        self.client.cancel_all_goals()
        self.empty_client.cancel_all_goals()
        try:
            pause_service = rospy.ServiceProxy(
                '/monitored_navigation/pause_nav',
                PauseResumeNav
            )
            pause_service(0)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()

    def card_callback(self, data):
        if self.pause == 1 and self.server.is_active():
            # call action server
            if data.data == 'near':
                self.odom_subscriber = None
                self.client_walking_interface.cancel_goal()
                rospy.loginfo("Therapist is close enough. Show continue button")
                self.empty_client.send_goal_and_wait(EmptyActionGoal())
                try:
                    pause_service = rospy.ServiceProxy(
                        '/monitored_navigation/pause_nav',
                        PauseResumeNav
                    )
                    pause_service(0)
                    self.pause = 0
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                self.client_walking_interface.send_goal(EmptyActionGoal())
                self.odom_subscriber = rospy.Subscriber("odom", Odometry,
                                                        self.odom_callback)

    def odom_callback(self, data):
        if self.begin == 0 and self.server.is_active():
            if self.counter >= 10:
                x = data.pose.pose.position.x - \
                    self.last_location.pose.pose.position.x
                y = data.pose.pose.position.y - \
                    self.last_location.pose.pose.position.y

                lenght = numpy.sqrt(x*x + y*y)

                if lenght >= self.distance:
                    # self.odom_subscriber.unregister()
                    rospy.loginfo("Reached %f meters" % self.distance)
                    try:
                        pause_service = rospy.ServiceProxy(
                            '/monitored_navigation/pause_nav',
                            PauseResumeNav
                        )
                        pause_service(1)
                        self.pause = 1
                        self.begin = 1
                        rospy.loginfo("Navigation paused")
                        s = rospy.ServiceProxy('/sound_player_service', PlaySoundService)
                        s.wait_for_service()
                        s("jingle_stop.mp3")
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                    # self.odom_subscriber = rospy.Subscriber("odom", Odometry, self.odom_callback)
                self.counter = 0
        else:
            self.last_location.pose.pose.position.x = data.pose.pose.position.x
            self.last_location.pose.pose.position.y = data.pose.pose.position.y
            self.begin = 0

        self.counter += 1



if __name__ == '__main__':
    rospy.init_node('guiding_server')
    server = GuidingServer(rospy.get_name())
    rospy.spin()
