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

generate_messages(
	DEPENDENCIES
	std_msgs
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
)

#############
## Install ##
#############

install(PROGRAMS
	scripts/diffdrive_exec.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
