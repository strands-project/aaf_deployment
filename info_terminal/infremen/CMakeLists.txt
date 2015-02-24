cmake_minimum_required(VERSION 2.8.3)
project(infremen)

find_package(catkin REQUIRED COMPONENTS  mongodb_store roscpp actionlib_msgs actionlib) 

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

catkin_package(CATKIN_DEPENDS roscpp message_runtime actionlib_msgs strands_executive_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(infremen src/infremen.cpp)

add_dependencies(infremen ${catkin_EXPORTED_TARGETS} mongodb_store)

add_library(infremen_frelement src/CFrelement.cpp)
add_library(infremen_timer src/CTimer.cpp)
add_library(infremen_frelementset src/CFrelementSet.cpp)

target_link_libraries(infremen_frelement infremen_timer)
target_link_libraries(infremen_frelementset infremen_frelement)

target_link_libraries (infremen
		${catkin_LIBRARIES}
		infremen_frelementset
)
