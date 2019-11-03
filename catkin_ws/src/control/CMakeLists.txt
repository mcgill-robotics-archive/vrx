cmake_minimum_required(VERSION 2.8.3)
project(control)

set(CMAKE_CXX_STANDARD 17)

find_package(
  catkin REQUIRED
  COMPONENTS
 	message_generation
	geometry_msgs
	std_msgs
  nav_msgs
	rospy
	pid
)

find_package(Boost REQUIRED)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
	FILES
	ThrusterCmd.msg
)

add_service_files(
  FILES
  Waypoint.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
  geometry_msgs
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
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

#############
## Install ##
#############

install(PROGRAMS
	scripts/diffdrive_exec.py
  scripts/translator.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
