cmake_minimum_required(VERSION 2.8.3)
project(robot_grasp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	message_generation
	roscpp
	rospy
	std_msgs
	sensor_msgs
)


generate_messages(
	DEPENDENCIES
    std_msgs
	sensor_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_grasp
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

