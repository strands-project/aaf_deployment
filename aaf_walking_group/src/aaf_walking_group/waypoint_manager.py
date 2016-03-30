#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from strands_navigation_msgs.msg import TopologicalMap
from topological_navigation.route_search import TopologicalRouteSearch


class WaypointManager():
    __resting_node_tag = "walking_group_resting_point"

    RESTING = "resting"
    INTERMEDIATE = "intermediate"
    ASKING = "asking"
    EXCLUDE = "exclude"
    PAGE = "page"
    RESTING_TAG = "resting_tag"

    def __init__(self, waypoints):
        rospy.loginfo("Waiting for topo map")
        topo_map = rospy.wait_for_message('/topological_map', TopologicalMap, timeout=10.0)
        rospy.loginfo("Got topo map")
        self.rsearch = TopologicalRouteSearch(topo_map)
        self.waypoints = waypoints
        self.chair = ''
        self.waypoint_names = [w[self.RESTING] for w in self.waypoints]
        self.goal_waypoint_idx = 0
        self.current_waypoint_index = 0
        self.route = {"route": [], "idx": 0, "topo_route": []}
        self.last_node = ''
        self.current_node = ''

        rospy.Subscriber("/current_node", String, self.node_cb)

    def node_cb(self, msg):
        if msg.data == 'none': return
        print msg.data, self.route["topo_route"]
        if msg.data not in self.route["topo_route"]: return
        if self.current_node != msg.data:
            self.last_node = self.current_node
            self.current_node = msg.data
        print self.last_node, self.current_node

    def contains(self, small, big):
        for i in xrange(len(big)-len(small)+1):
            for j in xrange(len(small)):
                if big[i+j] != small[j]:
                    break
            else:
                return i, i+len(small)
        raise ValueError("List not contained")

    def set_index(self, i):
        self.goal_waypoint_idx = i

    def get_index(self):
        return self.goal_waypoint_idx

    def get_waypoints(self):
        return self.waypoints

    def get_waypoint(self, i):
        return self.get_waypoints()[i]

    def get_current_waypoint(self):
        return self.get_waypoint(self.goal_waypoint_idx)

    def get_current_resting_tag(self):
        return self.get_current_waypoint()[self.RESTING_TAG]

    def get_page(self):
        return self.get_current_waypoint()[self.PAGE]

    def get_next_waypoint(self):
        return self.get_waypoint(self.goal_waypoint_idx+1)

    def get_prev_waypoint(self):
        idx = self.goal_waypoint_idx-1 if self.goal_waypoint_idx-1 >= 0 else 0
        return self.get_waypoint(idx)

    def advance(self):
        self.goal_waypoint_idx += 1
        return self.get_current_waypoint()

    def reverse(self):
        self.goal_waypoint_idx = self.goal_waypoint_idx-1 if self.goal_waypoint_idx-1 >= 0 else 0
        return self.get_current_waypoint()

    def set_resting_chair(self, chair):
        self.chair = chair

    def get_resting_chair(self):
        return self.chair

    def get_current_resting_waypoint(self):
        return self.get_current_waypoint()[self.RESTING]

    def get_current_asking_waypoint(self):
        return self.get_current_waypoint()[self.ASKING]

    def get_resting_waypoints(self):
        return [x["resting"] for x in self.get_waypoints()]

    def get_current_intermediate_waypoints(self):
        try:
            r = [w for u,w in sorted(self.get_current_waypoint()[self.INTERMEDIATE].items(),key=lambda to_int: int(to_int[0]))]
        except KeyError:
            r = []
        return r

    def get_all_intermediate_waypoints(self, idx=None):
        idx = self.get_index()+1 if idx == None else idx
        res = []
        for i in range(idx):
            try:
                e = self.get_waypoint(i)[self.EXCLUDE]
            except KeyError:
                e = []

            try:
                r = [w for u,w in sorted(self.get_waypoint(i)[self.INTERMEDIATE].items(),key=lambda to_int: int(to_int[0])) if w not in e]
            except KeyError:
                r = []

            res.extend(r)

        asking = self.get_current_waypoint()[self.ASKING]
        if asking not in res:
            res.append(asking)
        return res

    def create_route(self):
        r = self.get_all_intermediate_waypoints()
        rospy.loginfo("Pruning route")
        current_node = rospy.wait_for_message("/closest_node", String).data

        while not rospy.is_shutdown() and r[:-1]:
            rospy.loginfo("Testing route for doubling back")
            rospy.loginfo("Getting route from %s to %s via %s" % (current_node, r[-1], str(r[:-1])))
            route = self.rsearch.search_route(current_node, r[0]).source
            try:
                route.extend(self.rsearch.search_route(r[0], r[1]).source)
            except IndexError:
                pass
            rospy.loginfo("Found route %s" %str(route))
            if len(route) != len(set(route)):
                rospy.loginfo("Found duplicates, pruning first node")
                r = r[1:]
            else:
                rospy.loginfo("Found no duplicates, route is good!")
                break

        while not rospy.is_shutdown() and r[:-1]:
            rospy.loginfo("Testing route for already completed parts")
            rospy.loginfo("Getting route from %s to %s via %s" % (current_node, r[-1], str(r[:-1])))
            route = []
            for r1, r2 in zip(r[:-1],r[1:]):
                route.extend(self.rsearch.search_route(r1, r2).source)
            route_from_robot = [self.last_node, self.current_node]
            print "Robot route:", route_from_robot
            rospy.loginfo("Found route %s" %str(route))
            try:
                self.contains(route_from_robot, route)
            except ValueError:
                rospy.loginfo("Found no completed parts, route is good!")
                break
            else:
                rospy.loginfo("Found already completed parts, pruning first node")
                r = r[1:]
        idx = 0 if not self.route["route"] == r else self.route["idx"]
        route = []
        tmp = [current_node]
        tmp.extend(r)
        print tmp
        for r1, r2 in zip(tmp[:-1],tmp[1:]):
            print r1, r2
            route.extend(self.rsearch.search_route(r1, r2).source)
            print route
        if route[-1] != r[-1]: route.append(r[-1])
        self.route = {"route": r, "idx": idx, "topo_route": route}

    def get_route_to_current_waypoint(self):
        return self.route

    def advance_on_route(self):
        self.route["idx"] = self.route["idx"] + 1
        return self.route["route"][self.route["idx"]]

    def reverse_on_route(self):
        self.route["idx"] = self.route["idx"] - 1 if self.route["idx"] > 0 else 0
        return self.route["route"][self.route["idx"]]

    def get_current_waypoint_in_route(self):
        return self.route["route"][self.route["idx"]]