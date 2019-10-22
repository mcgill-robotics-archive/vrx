FROM ros:melodic-ros-core-bionic
ENV DEBIAN_FRONTEND noninteractive

ENV ROBOT vrx
ENV IAMROBOT true

RUN apt-get update && apt-get install git mercurial curl apt-utils vim \
                      python-pip python3-pip sudo cmake ruby libeigen3-dev \
                      pkg-config protobuf-compiler ros-melodic-pid \
                      ros-melodic-xacro wget ros-melodic-geographic-msgs -y

RUN pip install catkin_tools

RUN git config --global push.default simple && \
  git config --global url.'https://github.com/'.insteadOf 'git@github.com:' && \
  git config --global credential.helper 'cache --timeout=1800'

RUN git clone https://github.com/mcgill-robotics/compsys.git -b non-interactive /opt/compsys
WORKDIR /opt/compsys

RUN git clone https://github.com/mcgill-robotics/tools.sh.git /opt/tools.sh

RUN ./setup/zsh/install
RUN ./setup/config/install

WORKDIR /opt

ADD tools/. /opt/vrx/tools/
RUN ./vrx/tools/gazebo/upgrade-gazebo.sh
RUN apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-dev -y

ADD catkin_ws/. vrx/catkin_ws/

WORKDIR vrx/catkin_ws/
RUN catkin clean -b --yes
RUN /bin/bash -c "source /opt/ros/melodic/setup.sh && catkin build"

WORKDIR /opt/vrx
RUN sudo apt-get install tmuxinator -y
ADD config/. config/
ADD run.sh ./
ENTRYPOINT ["./run.sh"]
