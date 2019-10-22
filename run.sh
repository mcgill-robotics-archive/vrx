#!/bin/bash

cd /opt/vrx

source /opt/ros/melodic/setup.bash
source /opt/vrx/catkin_ws/devel/setup.bash

cd catkin_ws
catkin build
source devel/setup.bash

roslaunch src/control/launch/control.launch
roslaunch src/planner/launch/planner.launch
