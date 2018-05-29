cmake_minimum_required(VERSION 2.8.3)
project(robot_grasp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	message_generation
	pcl_conversions
	pcl_ros
	roscpp
	rospy
	genmsg
	roslint
	std_msgs
	sensor_msgs
     #   VTK 
)

catkin_python_setup()

add_message_files(
	FILES
	DetectedObject.msg
	DetectedObjectsArray.msg
)

add_service_files(
  FILES
  GetNormals.srv
  GetFloatArrayFeature.srv
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
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
# include(${VTK_USE_FILE})

# add_executable(cloud_transformer src/cloud_transformer.cpp)
# add_executable(feature_extractor src/feature_extractor.cpp)
add_executable(PCL_Processor src/PCL_Processor.cpp)
add_executable(BoundingBox src/BoundingBox.cpp)


#add_dependencies(cloud_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#add_dependencies(feature_extractor robot_grasp_gencpp)
add_dependencies(PCL_Processor robot_grasp_gencpp)
add_dependencies(BoundingBox robot_grasp_gencpp)

#target_link_libraries(cloud_transformer ${catkin_LIBRARIES})
#target_link_libraries(feature_extractor ${catkin_LIBRARIES})
target_link_libraries(PCL_Processor ${catkin_LIBRARIES})
target_link_libraries(BoundingBox ${catkin_LIBRARIES})

roslint_cpp(src/cloud_transformer.cpp)

