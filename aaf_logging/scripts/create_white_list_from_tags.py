#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest


if __name__ == "__main__":
    rospy.init_node("create_white_list_from_tags")
    try:
        rospy.loginfo("Waiting for tagged nodes service")
        s = rospy.ServiceProxy("/topological_map_manager/get_tagged_nodes", GetTaggedNodes)
        s.wait_for_service()
        rospy.loginfo(" ... calling service")
        nodes = s(GetTaggedNodesRequest(tag=rospy.get_param("~tag", "recording_white_list")))
    except rospy.ServiceException as e:
        rospy.logfatal(e)
        exit

    print nodes
    f = open(rospy.get_param("~file","white_list.yaml"), "w")
    yaml.dump({"nodes": nodes.nodes}, f, default_flow_style=False)
    f.close()