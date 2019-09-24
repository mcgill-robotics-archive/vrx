cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
  	message_generation
	rospy
	pid
	std_msgs
)

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
	geometry_msgs
  message_generation
	rospy
	pid
	std_msgs
  tf2_ros
)

#############
## Install ##
#############

install(PROGRAMS
	scripts/diffdrive_exec.py
  scripts/translator.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
