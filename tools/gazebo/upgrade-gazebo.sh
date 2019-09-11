#!/bin/bash
# Upgrades Gazebo 9 to the latest version directly from the OSRF repositories in
# order to get access to bug fixes that are otherwise unavailable in the default
# ROS repositories. ROS compatibility should be unaffected.

# Add OSRF Gazebo repository.
repo=http://packages.osrfoundation.org/gazebo/ubuntu-stable
sudo sh -c \
  "echo deb ${repo} bionic main > /etc/apt/sources.list.d/gazebo-stable.list"

# Setup keys.
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update.
sudo apt-get -y update
sudo apt-get -y upgrade gazebo9 libgazebo9-dev

