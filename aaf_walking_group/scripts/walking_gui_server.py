#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#import actionlib
import message_filters
import time
import math
import tf
from dynamic_reconfigure.client import Client as DynClient
from dynamic_reconfigure.server import Server as DynServer
from aaf_walking_group.cfg import LegibilityConfig
from threading import Thread
import strands_webserver.client_utils as client_utils
#from aaf_walking_group.msg import EmptyAction
from nav_msgs.msg import Path
from strands_navigation_msgs.msg import TopologicalMap, TopologicalRoute
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String



class WalkingInterfaceServer(object):

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.display_no = rospy.get_param("~display", 0)
        self.start_time = 0
        self.route = TopologicalRoute()
        self.coordinates = PoseArray()
        self.previous_direction = ""

        rospy.loginfo("%s: Starting walking interface.", name)

        self.head_pub = rospy.Publisher(
            '/head/commanded_state',
            JointState,
            queue_size=10)

        self.res_pub = rospy.Publisher(
            '~direction',
            String,
            queue_size=10,
            latch=True)

        self.head = JointState()
        self.head.name = ["HeadPan", "HeadTilt"]

        rospy.loginfo("%s: Waiting for topological map...", name)
        self.topological_nodes = rospy.wait_for_message(
            '/topological_map',
            TopologicalMap)
        rospy.loginfo("%s: ... done", name)

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

        self.thread = None
        self.dyn_client = None

        self.indicators = True
        self.web_page   = True
        self.move_head  = True
        self.dyn_srv = DynServer(LegibilityConfig, self.dyn_callback)

        rospy.loginfo("%s: ... all done.", name)

    def dyn_callback(self, config, level):
        self.indicators = config["indicators"]
        self.web_page   = config["web_page"]
        self.move_head  = config["move_head"]
        self.turn_angle = config["turn_angle"]

        if self.indicators and not self.dyn_client:
            rospy.loginfo("Waiting for dynamic reconfigure server for 1 sec...")
            try:
                self.dyn_client = DynClient('/EBC', timeout=1.0)
                rospy.loginfo(" ... done")
            except rospy.ROSException as e:
                rospy.logwarn(e)
        elif not self.indicators:
            self.dyn_client = None

        self.show_direction(self.previous_direction)

        return config

    def filter_callback(self, path, pose):
        self.start_time = time.time()

        robot_angle = tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])

        yDiff = path.poses[-1].pose.position.y - pose.pose.position.y
        xDiff = path.poses[-1].pose.position.x - pose.pose.position.x

        dist = math.sqrt(xDiff*xDiff + yDiff*yDiff)

        direction = "straight"

        if dist > 1.0:
            angle = robot_angle[-1] - math.atan2(yDiff,xDiff)

            while angle > math.pi:
                angle -= 2*math.pi

            while angle < -math.pi:
                angle += 2*math.pi

            #print math.degrees(angle)

            if abs(math.degrees(angle)) >= self.turn_angle:
                if angle > 0:
                    direction = "right"
                else:
                    direction = "left"

        self.visualise(direction)

    def route_callback(self, data):
        #search for waypouint and store coordinates
        print data.nodes
        for i in data.nodes:
            for j in self.topological_nodes.nodes:
                if i == j.name:
                    self.coordinates.poses.append(j.pose)

    def visualise(self, direction):
        if time.time() - self.start_time > 5:#reconfigurable parameter
            direction = "stop"
            if self.previous_direction != direction:
                rospy.loginfo("STOP")
                self.previous_direction = "stop"
                self.show_direction(direction)
        else:
            if self.previous_direction != direction:
                if direction == 'right':
                    rospy.loginfo("Moving right...")
                    self.previous_direction = "right"
                    self.show_direction(direction)

                elif direction == 'left':
                    rospy.loginfo("Moving left...")
                    self.previous_direction = "left"
                    self.show_direction(direction)

                else:
                    rospy.loginfo("Moving straight...")
                    self.previous_direction = "straight"
                    self.show_direction(direction)

    def show_direction(self, direction):
        direction = direction if not direction == "" else "stop"
        self.res_pub.publish(direction)
        if self.web_page:
            self.display(direction)
        if self.indicators:
            self.indicate(direction)
        if self.move_head:
            self.turn_head(direction)

    def display(self, direction):
        client_utils.display_relative_page(self.display_no, direction+".html")

    def turn_head(self, direction):
        self.head.header.stamp = rospy.Time.now()
        if direction == "left":
            self.head.position = [30, 0]
        elif direction == "right":
            self.head.position = [-30, 0]
        else:
            self.head.position = [0, 0]
        self.head_pub.publish(self.head)

    def indicate(self, direction):
        self.indicating = False
        if self.thread:
            self.thread.join()

        if direction in ["left", "right"]:
            self.indicating = True
            self.thread = Thread(target=self.blink, args=(direction,))
            self.thread.start()

    def switchIndicator(self, isOn, side):
        if side == "left":
            params = {'Port0_5V_Enabled' : isOn}
        else:
            params = {'Port1_5V_Enabled' : isOn}
        if self.dyn_client:
            self.dyn_client.update_configuration(params)

    def blink(self, side):
        r = rospy.Rate(2)
        toggle = True

        while self.indicating and not rospy.is_shutdown():
            self.switchIndicator(toggle, side)
            toggle = not toggle
            r.sleep()

        self.switchIndicator(False, side)

if __name__ == '__main__':
    rospy.init_node('walking_interface_server')
    wserver = WalkingInterfaceServer(rospy.get_name())
    rospy.spin()
