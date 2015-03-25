cmake_minimum_required(VERSION 2.8.3)
project(aaf_waypoint_sounds)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    rospy
    pygame_managed_player
    mongodb_media_server
    message_generation
)

add_service_files(
    FILES
    WaypointSoundsService.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

#############
## Install ##
#############
install(PROGRAMS
scripts/aaf_waypoint_player.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
