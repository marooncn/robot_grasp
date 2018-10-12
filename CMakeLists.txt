cmake_minimum_required(VERSION 2.8.3)
project(robot_grasp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# find_package(Eigen REQUIRED)
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	message_generation
	roscpp
	rospy
	genmsg
	roslint
	std_msgs
	sensor_msgs
        cv_bridge
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
)


add_message_files(
	FILES
        real_detected_obj.msg
        real_detected_obj_array.msg
)
find_package(kinova_driver REQUIRED)
add_service_files(
   FILES
   calibration_transform.srv
   real_kinova_pick.srv
)

generate_messages(
	DEPENDENCIES
        std_msgs
	sensor_msgs
)

# generate_dynamic_reconfigure_options(
#   config/Pcl.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
   CATKIN_DEPENDS message_runtime
   DEPENDS Eigen
)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  lib
)

add_executable(transform src/transform.cpp)

target_link_libraries(transform ${catkin_LIBRARIES})

