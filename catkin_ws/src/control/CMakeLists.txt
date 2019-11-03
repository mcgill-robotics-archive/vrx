cmake_minimum_required(VERSION 2.8.3)
project(control)

set(CMAKE_CXX_STANDARD 17)

find_package(
  catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
 	message_generation
	geometry_msgs
	std_msgs
  nav_msgs
  rospy
  pid
  )

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  Waypoint.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/PIDParams.cfg
)
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  message_generation
  geometry_msgs
  std_msgs
  tf2_ros
  rospy
  pid
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  src/pid
  include
  )

add_executable(controller src/controller.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(controller ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
