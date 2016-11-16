cmake_minimum_required(VERSION 2.8.3)
project(infremen)

find_package(catkin REQUIRED COMPONENTS  mongodb_store message_generation roscpp strands_executive_msgs std_msgs std_srvs strands_navigation_msgs mongodb_store_msgs geometry_msgs dynamic_reconfigure info_task) 

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  InfremenResult.msg
  AtomicInteraction.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/infremen.cfg
)

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb -std=c++11")

catkin_package(CATKIN_DEPENDS roscpp strands_executive_msgs message_generation std_msgs std_srvs strands_navigation_msgs mongodb_store_msgs geometry_msgs mongodb_store info_task)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(infremen src/infremen.cpp)

add_dependencies(infremen ${catkin_EXPORTED_TARGETS} mongodb_store ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_genmsg infremen_generate_messages)

add_library(infremen_frelement src/CFrelement.cpp)
add_library(infremen_timer src/CTimer.cpp)
add_library(infremen_frelementset src/CFrelementSet.cpp)
add_library(infremen_graphlib src/CGraph.cpp)

target_link_libraries(infremen_frelement infremen_timer)
target_link_libraries(infremen_frelementset infremen_frelement)

target_link_libraries (infremen
		${catkin_LIBRARIES}
		infremen_frelementset
		infremen_graphlib
)


