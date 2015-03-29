cmake_minimum_required(VERSION 2.8.3)
project(aaf_walking_group)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  music_player
  geometry_msgs
  message_generation
  mongodb_store
  nav_msgs
  roscpp
  roslib
  rospy
  smach
  smach_ros
  std_msgs
  std_srvs
  strands_navigation_msgs
  strands_webserver
  topological_navigation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  GetMediaId.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Interface.action
  Empty.action
  Guiding.action
  StateMachine.action
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
  scripts/positionOfCard.py
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

install(DIRECTORY www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
