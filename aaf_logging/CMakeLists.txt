cmake_minimum_required(VERSION 2.8.3)
project(aaf_logging)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  std_msgs
  topological_logging_manager
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  DIRECTORY action
  FILES
  Empty.action
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
catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)