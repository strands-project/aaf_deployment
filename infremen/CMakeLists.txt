cmake_minimum_required(VERSION 2.8.3)
project(infremen)

find_package(catkin REQUIRED COMPONENTS  mongodb_store roscpp genmsg actionlib_msgs actionlib) 

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

generate_messages(
	#   DEPENDENCIES actionlib_msgs 
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(infremen src/infremen.cpp)

add_dependencies(infremen ${catkin_EXPORTED_TARGETS} mongodb_store)

target_link_libraries (infremen
		${catkin_LIBRARIES}
)
