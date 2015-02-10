#!/usr/bin/env python

import rospy
import roslib
import smach
import smach_ros
import std_msgs.msg
import strands_webserver
import strands_webserver.client_utils
from mongodb_store.message_store import MessageStoreProxy
from aaf_walking_group.entertain import Entertain
from aaf_walking_group.guide_interface import GuideInterface
from aaf_walking_group.guiding import Guiding
import json
import pprint


def loadConfig(dataset_name, collection_name="aaf_walking_group", meta_name="waypoint_set"):
    msg_store = MessageStoreProxy(collection=collection_name)
    query_meta = {}
    query_meta[meta_name] = dataset_name
    if len(msg_store.query(std_msgs.msg.String._type, {}, query_meta)) == 0:
        rospy.logerr("Desired data set '"+dataset_name+"' not in datacentre.")
        raise Exception("Can't find data in datacentre.")
    else:
        message = msg_store.query(std_msgs.msg.String._type, {}, query_meta)
        return json.loads(message[0][0].data)


def main():
    rospy.init_node('walking_group_smach')

    # Get parameters
    #topomap = rospy.get_param("~topological_map", "")
    waypointset_name = rospy.get_param("~mongodb_params/waypointset_name", "")
    waypointset_collection = rospy.get_param("~mongodb_params/waypointset_collection", "aaf_walking_group")
    waypointset_meta = rospy.get_param("~mongodb_params/waypointset_meta", "waypoint_set")
    if waypointset_name == "":
        rospy.logfatal("Missing parameters.")
        rospy.logfatal("Please run with _mongodb_params/waypointset_name:=<waypointset_name>")
        return

    waypointset = loadConfig(waypointset_name, collection_name=waypointset_collection, meta_name=waypointset_meta)
    pprint.pprint(waypointset)

    # Setting http root
#    http_root = roslib.packages.get_pkg_dir('aaf_walking_group') + '/www'
#    strands_webserver.client_utils.set_http_root(
#        http_root
#    )

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    sm.userdata.current_waypoint = "waypoint11"  # TODO: Get start point
    sis = smach_ros.IntrospectionServer(
        'walking_group_state_machine',
        sm,
        '/walking_group_machine'
    )
    sis.start()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            'ENTERTAIN',
            Entertain(),
            transitions={
                'key_card': 'GUIDE_INTERFACE'
            },
            remapping={'current_waypoint' : 'current_waypoint'}
        )
        smach.StateMachine.add(
            'GUIDE_INTERFACE',
            GuideInterface(waypointset),
            transitions={
                'move_to_point': 'GUIDING',
                'aborted': 'ENTERTAIN',
                'killall': 'preempted'
            },
            remapping={'current_waypoint' : 'current_waypoint'}
        )
        smach.StateMachine.add(
            'GUIDING',
            Guiding(),
            transitions={
                'reached_point': 'ENTERTAIN',
                'reached_final_point': 'succeeded',
                'key_card': 'GUIDE_INTERFACE'
            },
            remapping={'waypoint' : 'waypoint'}
        )

    # Execute SMACH plan
    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
