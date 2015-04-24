#!/usr/bin/env python

import rospy
import roslib
import rosparam
import smach
import smach_ros
import std_msgs.msg
from dynamic_reconfigure.client import Client as DynClient
from std_srvs.srv import Empty, EmptyResponse
import strands_webserver
import strands_webserver.client_utils
from mongodb_store.message_store import MessageStoreProxy
from aaf_walking_group.entertain import Entertain
from aaf_walking_group.guide_interface import GuideInterface
from aaf_walking_group.guiding import Guiding
from aaf_walking_group.reached_resting_point import RestingPoint
from aaf_walking_group.msg import GuidingAction, EmptyAction, StateMachineAction
from aaf_walking_group.srv import GetMediaId
from aaf_waypoint_sounds.srv import WaypointSoundsService, WaypointSoundsServiceRequest
from aaf_walking_group.utils import PTU, Gaze, RecoveryReconfigure
from music_player.srv import MusicPlayerService
from sound_player_server.srv import PlaySoundService
from walking_group_recovery.srv import ToggleWalkingGroupRecovery
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest, GetTaggedNodesResponse
import actionlib
import json
import pprint


class WalkingGroupStateMachine(object):
    def __init__(self, name):
        rospy.loginfo("Creating " + name + " server...")
        self._as = actionlib.SimpleActionServer(
            name,
            StateMachineAction,
            self.execute,
            auto_start=False
        )
        self._as.register_preempt_callback(self.preempt_callback)

        rospy.loginfo("Creating guiding client...")
        nav_client = actionlib.SimpleActionClient("guiding", GuidingAction)
        nav_client.wait_for_server()
        rospy.loginfo(" ... done")
        rospy.loginfo("Creating wait client...")
        wait_client = actionlib.SimpleActionClient("wait_for_participant", EmptyAction)
        wait_client.wait_for_server()
        rospy.loginfo(" ... done")
        rospy.loginfo("Waiting for recovery toggle services...")
        rospy.loginfo(" ... images")
        self.rec_srv = rospy.ServiceProxy('/toggle_walking_group_recovery', ToggleWalkingGroupRecovery)
        self.rec_srv.wait_for_service()
        rospy.loginfo(" ... done")
        rospy.loginfo("Waiting for media server services...")
        rospy.loginfo(" ... images")
        s = rospy.ServiceProxy('/aaf_walking_group/image_server/get_id', GetMediaId)
        s.wait_for_service()
        rospy.loginfo(" ... video")
        s = rospy.ServiceProxy('/aaf_walking_group/video_server/get_id', GetMediaId)
        s.wait_for_service()
        rospy.loginfo(" ... sound")
        rospy.loginfo("  ... music")
        s = rospy.ServiceProxy('/music_player_service', MusicPlayerService)
        s.wait_for_service()
        rospy.loginfo("  ... jingles")
        s = rospy.ServiceProxy('/sound_player_server/sound_player_service', PlaySoundService)
        s.wait_for_service()
        rospy.loginfo("  ... waypoints")
        s = rospy.ServiceProxy('/aaf_waypoint_sounds_service', WaypointSoundsService)
        s.wait_for_service()
        rospy.loginfo(" ... done")
        # Get parameters
        self.display_no = rospy.get_param("~display_no", 0)
        self.waypointset_name = rospy.get_param("~mongodb_params/waypointset_name", "")
        self.waypointset_collection = rospy.get_param("~mongodb_params/waypointset_collection", "aaf_walking_group")
        self.waypointset_meta = rospy.get_param("~mongodb_params/waypointset_meta", "waypoint_set")
        if self.waypointset_name == "":
            rospy.logfatal("Missing parameters.")
            rospy.logfatal("Please run with _mongodb_params/waypointset_name:=<waypointset_name>")
            return

        self.preempt_srv = None

        self.ptu = PTU()
        self.gaze = Gaze()
        self.recovery = RecoveryReconfigure(
            name="/monitored_navigation/recover_states/",
            whitelist=rosparam.load_file(rospy.get_param("~recovery_whitelist"))[0][0]["recover_states"]
        )
        rospy.loginfo("Creating dynamic reconfigure clients")
        self.han_dyn_client = DynClient(
            "/human_aware_navigation"
        )
        self.mov_dyn_client = DynClient(
            "/move_base/DWAPlannerROS"
        )
        rospy.loginfo(" ...done")
        self.get_current_dyn_settings()

        rospy.loginfo(" ... starting " + name)
        self._as.start()
        rospy.loginfo(" ... started " + name)

    def get_current_dyn_settings(self):
        gazing = rospy.get_param("/human_aware_navigation/gaze_type")
        angle = round(rospy.get_param("/human_aware_navigation/detection_angle"),2)
        self.han_param = {
            'gaze_type': gazing,
            'detection_angle': angle
        }
        rospy.loginfo("Found following default values for human_aware_navigation: %s", self.han_param)
        max_vel_x = round(rospy.get_param("/move_base/DWAPlannerROS/max_vel_x"), 2)
        max_trans_vel = round(rospy.get_param("/move_base/DWAPlannerROS/max_trans_vel"),2)
        self.mov_param = {
            "max_vel_x": max_vel_x,
            "max_trans_vel": max_trans_vel
        }
        rospy.loginfo("Found following default values for move_base: %s", self.mov_param)

    def execute(self, goal):
        rospy.loginfo("Starting state machine")

        resting_points = []
        try:
            rospy.loginfo("Creating get tagged nodes service proxy and waiting ...")
            s = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', GetTaggedNodes)
            s.wait_for_service()
            rospy.loginfo(" ... calling get tagged nodes service")
            req = GetTaggedNodesRequest(tag='walking_group_resting_point')
            res = s(req)
            resting_points = res.nodes
            rospy.loginfo(" ... called get tagged nodes recovery")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logfatal("get tagged nodes service call failed: %s" % e)
            self._as.set_aborted()
            return

        try:
            rospy.loginfo("Creating recovery toggle service proxy and waiting ...")
            self.rec_srv.wait_for_service()
            rospy.loginfo(" .. calling recovery toggle service")
            self.rec_srv(True)
            rospy.loginfo(" .. started walking group recovery")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn("waypoint sound service call failed: %s" % e)
        #Save current recovery behaviour states
        self.recovery.save_current_configuration()
        #and reconfigure
        self.recovery.reconfigure(RecoveryReconfigure.SET)

        # Setting http root
        http_root = roslib.packages.get_pkg_dir('aaf_walking_group') + '/www'
        strands_webserver.client_utils.set_http_root(http_root)

        self.ptu.turnPTU(-180, 10)
        dyn_param = {
            'timeout': 0.0,
            'gaze_type': 1,
            'detection_angle': 20.0
        }
        try:
            self.han_dyn_client.update_configuration(dyn_param)
        except rospy.ServiceException as e:
            rospy.logerr("Caught service exception: %s", e)

        self.preempt_srv = rospy.Service('/walking_group/cancel', Empty, self.preempt_srv_cb)
        self.waypointset = self.loadConfig(self.waypointset_name, collection_name=self.waypointset_collection, meta_name=self.waypointset_meta)
        pprint.pprint(self.waypointset)

        dyn_param = {
            'max_vel_x': self.waypointset[goal.group]["speed"],
            'max_trans_vel': self.waypointset[goal.group]["speed"]
        }
        try:
            self.mov_dyn_client.update_configuration(dyn_param)
        except rospy.ServiceException as e:
            rospy.logerr("Caught service exception: %s", e)

        resting_points_dict = {k: i for k,i in self.waypointset[goal.group]["waypoints"].iteritems() if i in resting_points}
        pprint.pprint(resting_points)
        pprint.pprint(resting_points_dict)
        try:
            rospy.loginfo("Creating waypoint sound service proxy and waiting ...")
            s = rospy.ServiceProxy('aaf_waypoint_sounds_service', WaypointSoundsService)
            s.wait_for_service()
            rospy.loginfo(" .. calling waypoint sound service")
            s(WaypointSoundsServiceRequest.RESUME)
            rospy.loginfo(" .. started waypoint sound service")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn("waypoint sound service call failed: %s" % e)

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        self.sm.userdata.current_waypoint = self.waypointset[goal.group]["waypoints"][str(min([int(x) for x in self.waypointset[goal.group]["waypoints"].keys()]))]
        self.sm.userdata.play_music = True
        sis = smach_ros.IntrospectionServer(
            'walking_group_state_machine',
            self.sm,
            '/walking_group_machine'
        )
        sis.start()
        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add(
                'ENTERTAIN',
                Entertain(self.display_no, self.gaze),
                transitions={
                    'key_card': 'GUIDE_INTERFACE',
                    'killall': 'preempted'
                },
                remapping={'current_waypoint' : 'current_waypoint', 'play_music' : 'play_music'}
            )
            smach.StateMachine.add(
                'GUIDE_INTERFACE',
                GuideInterface(resting_points_dict),
                transitions={
                    'move_to_point': 'GUIDING',
                    'aborted': 'ENTERTAIN',
                    'killall': 'preempted'
                },
                remapping={'current_waypoint' : 'current_waypoint', 'play_music' : 'play_music'}
            )
            smach.StateMachine.add(
                'GUIDING',
                Guiding(waypoints=self.waypointset[goal.group]["waypoints"], distance=self.waypointset[goal.group]["stopping_distance"], resting_points=resting_points),
                transitions={
                    'reached_point': 'RESTING_CONT',
                    'reached_final_point': 'succeeded',
                    'continue': 'GUIDING',
                    'key_card': 'GUIDE_INTERFACE',
                    'killall': 'preempted'
                },
                remapping={'waypoint' : 'waypoint', 'play_music' : 'play_music'}
            )
            smach.StateMachine.add(
                'RESTING_CONT',
                RestingPoint(self.display_no,self.waypointset[goal.group]["waypoints"]),
                transitions={
                    'rest': 'ENTERTAIN',
                    'continue': 'GUIDING',
                    'key_card': 'GUIDE_INTERFACE',
                    'killall': 'preempted'
                },
                remapping={'current_waypoint' : 'current_waypoint', 'play_music' : 'play_music'}
            )

        # Execute SMACH plan
        self.sm.execute()

        sis.stop()
        self.preempt_srv.shutdown()
        self.ptu.turnPTU(0, 0)
        self.recovery.reconfigure(RecoveryReconfigure.RESET)
        try:
            self.han_dyn_client.update_configuration(self.han_param)
        except rospy.ServiceException as e:
            rospy.logerr("Caught service exception: %s", e)
        try:
            self.mov_dyn_client.update_configuration(self.mov_param)
        except rospy.ServiceException as e:
            rospy.logerr("Caught service exception: %s", e)
        try:
            rospy.loginfo("Creating waypoint sound service proxy and waiting ...")
            s = rospy.ServiceProxy('aaf_waypoint_sounds_service', WaypointSoundsService)
            s.wait_for_service()
            rospy.loginfo(" ... calling waypoint sound service")
            s(WaypointSoundsServiceRequest.PAUSE)
            rospy.loginfo(" ... stopped waypoint sound service")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn("Service call failed: %s" % e)
        try:
            rospy.loginfo("Creating recovery toggle service proxy and waiting ...")
            self.rec_srv.wait_for_service()
            rospy.loginfo(" .. calling recovery toggle service")
            self.rec_srv(False)
            rospy.loginfo(" .. stopped walking group recovery")
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logwarn("Service call failed: %s" % e)
        if not self._as.is_preempt_requested() and self._as.is_active():
            self._as.set_succeeded()
        else:
            self._as.set_preempted()

        strands_webserver.client_utils.display_relative_page(self.display_no, "ende.html")

    def preempt_callback(self):
        rospy.logwarn("Walking group preempt requested")
        self.sm.request_preempt()

    def preempt_srv_cb(self, req):
        self.preempt_callback()
        return EmptyResponse()

    def loadConfig(self, dataset_name, collection_name="aaf_walking_group", meta_name="waypoint_set"):
        msg_store = MessageStoreProxy(collection=collection_name)
        query_meta = {}
        query_meta[meta_name] = dataset_name
        if len(msg_store.query(std_msgs.msg.String._type, {}, query_meta)) == 0:
            rospy.logerr("Desired data set '"+meta_name+": "+dataset_name+"' not in datacentre.")
            raise Exception("Can't find data in datacentre.")
        else:
            message = msg_store.query(std_msgs.msg.String._type, {}, query_meta)
            return json.loads(message[0][0].data)


if __name__ == '__main__':
    rospy.init_node('walking_group_smach')
    w = WalkingGroupStateMachine(rospy.get_name())
    rospy.spin()
