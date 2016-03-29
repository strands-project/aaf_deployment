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
        self.route = {"route": [], "idx": 0}

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

    def get_all_intermediate_waypoints(self):
        res = []
        for i in range(self.get_index()+1):
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
        idx = 0 if not self.route["route"] == r else self.route["idx"]
        self.route = {"route": r, "idx": idx}

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