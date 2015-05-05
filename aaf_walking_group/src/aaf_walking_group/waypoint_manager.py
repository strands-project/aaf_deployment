#!/usr/bin/env python
# -*- coding: utf-8 -*-


class WaypointManager():
    RESTING = "resting"
    INTERMEDIATE = "intermediate"
    ASKING = "asking"

    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.route = {"route": [], "idx": 0}

    def set_index(self, i):
        self.current_waypoint_idx = i

    def get_index(self):
        return self.current_waypoint_idx

    def get_waypoints(self):
        return self.waypoints

    def get_waypoint(self, i):
        return self.get_waypoints()[i]

    def get_current_waypoint(self):
        return self.get_waypoint(self.current_waypoint_idx)

    def get_next_waypoint(self):
        return self.get_waypoint(self.current_waypoint_idx+1)

    def get_prev_waypoint(self):
        idx = self.current_waypoint_idx-1 if self.current_waypoint_idx-1 >= 0 else 0
        return self.get_waypoint(idx)

    def advance(self):
        self.current_waypoint_idx += 1
        return self.get_current_waypoint()

    def reverse(self):
        self.current_waypoint_idx = self.current_waypoint_idx-1 if self.current_waypoint_idx-1 >= 0 else 0
        return self.get_current_waypoint()

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

    def create_route(self):
        r = [x for x in self.get_current_intermediate_waypoints()]
        r.append(self.get_current_asking_waypoint())
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