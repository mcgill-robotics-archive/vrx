cmake_minimum_required(VERSION 2.8.3)
project(vrx_sim)

find_package(catkin REQUIRED COMPONENTS
  vrx_gazebo
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS
   vrx_gazebo
   wamv_gazebo
   wave_gazebo
   xacro
   gazebo_dev
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Generate world files from xacro and install
xacro_add_files(
  worlds/obstacle_demo.world.xacro
  INORDER INSTALL DESTINATION worlds
)

#############
## Testing ##
#############

