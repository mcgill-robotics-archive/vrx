#!/bin/bash
#
# Setup McGill Robotics Drone workspace
#
# shellcheck disable=SC1090
#

# Exit on first error
set -e

header=$(tput setab 1; tput setaf 0)
section=$(tput bold; tput setaf 2)
warning=$(tput bold; tput setaf 1)
reset=$(tput sgr0)

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Header
echo
echo "${header} Welcome to McGill Robotics Drone ${reset}"

# Verify running on Ubuntu 16.04
if [[ $(lsb_release -sc) != "bionic" ]]; then
  echo "${warning}CompSys only supports Ubuntu 18.04${reset}"
  exit -1
fi

# Check and ask for root power
if [[ "${EUID}" -eq 0 ]]; then
  echo "${warning}Please DO NOT run this script as root${reset}"
  exit -1
fi
sudo -v

function _pretty_path {
  realpath --relative-base="${DIR}" "${1}"
}

function _symlink {
  echo 'Symlinking '"$(_pretty_path "${1}")"' to '"$(_pretty_path "${2}")"'...'
  if [[ "${3}" == "root" ]]; then
    sudo ln -s "${1}" "${2}"
  else
    ln -s "${1}" "${2}"
  fi
}

function symlink_dir  {
  if [[ ! -e "${2}" ]]; then
    if [[ -L "${2}" ]]; then
      echo 'Removing broken link '"$(_pretty_path "${2}")"'...'
      sudo rm -rf "${2}"
    fi
    _symlink "${@}"
  elif [[ ! -L "${2}" ]]; then
    sudo rm -rf "${2}"
    _symlink "${@}"
  fi
}

function symlink_file  {
  if [[ ! -e "${2}" ]]; then
    if [[ -L "${2}" ]]; then
      echo 'Removing broken link '"$(_pretty_path "${2}")"'...'
      sudo rm -f "${2}"
    fi
    _symlink "${@}"
  fi
}

VRX_DIR="${DIR}/catkin_ws/src/vrx"

sudo apt-get install -y cmake mercurial git ruby libeigen3-dev pkg-config \
                        protobuf-compiler

./tools/gazebo/upgrade-gazebo.sh

if [[ ! -d  ${VRX_DIR} ]]; then
  echo "${section} Cloning VRX repo... ${reset}"
  hg clone https://bitbucket.org/osrf/vrx ${VRX_DIR}
fi

update_repo.sh

pushd catkin_ws
catkin build
popd






