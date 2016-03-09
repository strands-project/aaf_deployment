cmake_minimum_required(VERSION 2.8.3)
project(aaf_walking_group)

find_package(catkin REQUIRED COMPONENTS
  aaf_waypoint_sounds
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  flir_pantilt_d46
  geometry_msgs
  message_generation
  message_filters
  mongodb_media_server
  mongodb_store
  move_base_msgs
  music_player
  nav_msgs
  roscpp
  roslaunch_axserver
  roslib
  rospy
  smach
  smach_ros
  sensor_msgs
  sound_player_server
  std_msgs
  std_srvs
  strands_executive_msgs
  strands_gazing
  strands_navigation_msgs
  strands_webserver
  tf
  topological_navigation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

generate_dynamic_reconfigure_options(
  cfg/Legibility.cfg
)

add_service_files(
  FILES
  GetMediaId.srv
  Button.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Interface.action
  MapInterface.action
  Empty.action
  Guiding.action
  StateMachine.action
  WalkingGroup.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS mongodb_store roscpp rospy smach smach_ros std_msgs
  strands_webserver actionlib actionlib_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/aaf_walking_group_state_machine.py
  scripts/guiding_action.py
  scripts/insert_yaml.py
  scripts/interface_demo.py
  scripts/interface_server.py
  scripts/media_server.py
  scripts/positionOfCard.py
  scripts/start_walking_group.py
  scripts/wait_for_participant.py
  scripts/walking_gui_demo.py
  scripts/walking_gui_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY etc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
