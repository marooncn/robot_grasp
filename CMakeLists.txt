cmake_minimum_required(VERSION 2.8.3)
project(robot_grasp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(PCL_lib_DIR /opt/PCL/pcl/release/lib)


#find_package(Eigen REQUIRED)
#set(CMAKE_PREFIX_PATH "/opt/PCL/pcl/release")
#find_package(PCL 1.8 REQUIRED)
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	message_generation
#	pcl_conversions
#        pcl_ros
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

catkin_python_setup()
add_message_files(
	FILES
	DetectedObject.msg
	DetectedObjectsArray.msg
        Detected_obj_param.msg
        Detected_obj_param_Array.msg
        VFH.msg
        tramsform_obj_param.msg
        real_detected_obj.msg
        real_detected_obj_array.msg
)
find_package(kinova_driver REQUIRED)
add_service_files(
   FILES
   GetFloatArrayFeature.srv
   GetNormals.srv
   GetVfh.srv
   SprawnModel.srv
   Centroid_Compute.srv
   coordinate_transform.srv
   VFH_pick_place.srv
   G_pick_place.srv
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
#  INCLUDE_DIRS include
#  LIBRARIES robot_grasp
   CATKIN_DEPENDS message_runtime
   DEPENDS Eigen
#   pcl_conversions pcl_ros
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
 # ${PCL_include_DIR}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  lib
 # ${PCL_INCLUDE_DIRS}
)# calib_opencv.cpp
#include_directories(${kinova_driver_INCLUDE_DIRS})
#add_executable(VFH src/VFH.cpp)
#add_executable(VFH src/VFH.cpp)
#add_executable(centroid_compute src/centroid_compute.cpp)
#add_executable(pick_place_ src/pick_place_.cpp)
#add_executable(coordinates_transformer src/coordinates_transformer.cpp)
add_executable(transform src/transform.cpp)
#add_executable(Kinova_pick_place src/Kinova_pick_place.cpp)

#target_link_libraries(pick_place_ ${catkin_LIBRARIES} libboost_thread.so)
#target_link_libraries(VFH ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
#target_link_libraries(centroid_compute ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
#target_link_libraries(pick_place_ ${catkin_LIBRARIES} libboost_thread.so)
#target_link_libraries(coordinates_transformer ${catkin_LIBRARIES})
target_link_libraries(transform ${catkin_LIBRARIES})
#target_link_libraries(Kinova_pick_place ${catkin_LIBRARIES})

#roslint_cpp(src/cloud_transformer.cpp)


