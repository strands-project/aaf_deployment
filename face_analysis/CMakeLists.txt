cmake_minimum_required(VERSION 2.8.3)
project(face_analysis)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs

)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############



