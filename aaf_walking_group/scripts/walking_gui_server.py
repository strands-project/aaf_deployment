#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os

import rospy
import roslib
import actionlib
import message_filters
import time
import math
import tf

import strands_webserver.client_utils as client_utils
from aaf_walking_group.msg import EmptyAction
#from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path
from strands_navigation_msgs.msg import TopologicalMap, TopologicalRoute
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray



class WalkingInterfaceServer(object):

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.display_no = rospy.get_param("~display", 0)
        self.start_time = 0
        self.route = TopologicalRoute()
        self.coordinates = PoseArray()
        self.direction = "straight"
        self.previous_direction = ""

        self.head_pub = rospy.Publisher(
            '/head/commanded_state',
            JointState,
            queue_size=10)
            
        self.head = JointState()
        self.head.name = "HeadPan"
        
        self.topological_nodes = rospy.wait_for_message(
            '/topological_map',
            TopologicalMap)

        self.route_sub = rospy.Subscriber(
            "/topological_navigation/Route",
            TopologicalRoute,
            self.route_callback)
            
        self.path_sub = message_filters.Subscriber(
            "/move_base/NavfnROS/plan",
            Path)
            
        self.pose_sub = message_filters.Subscriber(
            "/pose_extractor/pose",
            PoseStamped)
            
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.path_sub, self.pose_sub],
            10,
            10)
            
        self.ts.registerCallback(self.filter_callback)
        
        #tell the webserver where it should look for web files to serve
        http_root = os.path.join(
            roslib.packages.get_pkg_dir("aaf_walking_group"),
            "www")
        client_utils.set_http_root(http_root)

        #Starting server
        rospy.loginfo("%s: Starting walking interface action server", name)
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            EmptyAction,
            execute_cb=self.executeCallback,
            auto_start=False
        )
        
        self._as.start()
        rospy.loginfo("%s: ...done.", name)
        
    
    def filter_callback(self, path, pose):
        self.start_time = time.time()
        
        robot_angle = tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        
        yDiff = path.poses[-1].pose.position.y - pose.pose.position.y
        xDiff = path.poses[-1].pose.position.x - pose.pose.position.x
        
        dist = math.sqrt(xDiff*xDiff + yDiff*yDiff)
        
        if dist > 1.0:
            angle = robot_angle[-1] - math.atan2(yDiff,xDiff)
        
            while angle > math.pi:
                angle -= 2*math.pi
            
            while angle < -math.pi:
                angle += 2*math.pi
            
            #print math.degrees(angle)
        
            if abs(math.degrees(angle)) < 30:
                self.direction = "straight"
            else:
                if angle > 0:
                    self.direction = "right"
                else:
                    self.direction = "left"
        else:
            self.direction = "straight"
                
        
        
    def route_callback(self, data):
        #search for waypouint and store coordinates
        print data.nodes
        for i in data.nodes:
            for j in self.topological_nodes.nodes:
                if i == j.name:
                    self.coordinates.poses.append(j.pose)

    def executeCallback(self, goal):
        while not self._as.is_preempt_requested():
            if time.time() - self.start_time > 5:#reconfigurable parameter
                self.direction = "stop"
                if self.previous_direction != self.direction:
                    client_utils.display_relative_page(self.display_no,
                                                       'stop.html')
                    rospy.loginfo("STOP")
                    #move head straight
                    self.head.position = 0
                    self.head_pub.publish(self.head)
                    self.previous_direction = "stop"
            else:
                if self.previous_direction != self.direction:
                    if self.direction == 'right':
                        client_utils.display_relative_page(self.display_no,
                                                           'turn_right.html')
                        #move head right
                        self.head.position = -80
                        self.head_pub.publish(self.head)
                        rospy.loginfo("Moving right...")
                        self.previous_direction = "right"
                    
                    elif self.direction == 'left':
                        client_utils.display_relative_page(self.display_no,
                                                           'turn_left.html')
                        #move head left
                        self.head.position = 80
                        self.head_pub.publish(self.head)
                        rospy.loginfo("Moving left...")
                        self.previous_direction = "left"
                        
                    else:
                        client_utils.display_relative_page(self.display_no,
                                                           'straight.html')
                        #move head straight
                        self.head.position = 0
                        self.head_pub.publish(self.head)
                        rospy.loginfo("Moving straight...")
                        self.previous_direction = "straight"
                
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            
    def _on_node_shutdown(self):
        self.client.cancel_all_goals()

if __name__ == '__main__':
    rospy.init_node('walking_interface_server')
    wserver = WalkingInterfaceServer(rospy.get_name())
    rospy.spin()
