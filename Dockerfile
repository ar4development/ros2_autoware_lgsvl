FROM ubuntu:20.04

ENV ROS_DISTRO=foxy
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ARG DEBIAN_FRONTEND=noninteractive

#### INSTALLING ROS2
# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

RUN apt update && apt install -y locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8

# https://stackoverflow.com/questions/44331836/apt-get-install-tzdata-noninteractive
# https://askubuntu.com/questions/876240/how-to-automate-setting-up-of-keyboard-configuration-package
RUN apt install -y --no-install-recommends tzdata && apt install -y keyboard-configuration

RUN apt install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt update && apt install -y ros-$ROS_DISTRO-desktop && apt install -y ros-$ROS_DISTRO-ros-base
RUN apt install -y python3-pip && pip3 install -U argcomplete && pip3 install -U pytest && pip3 install -U colcon-common-extensions vcstool

#### INSTALLING AUTOWARE.AUTO

RUN apt install -y git
RUN mkdir ad_stack
WORKDIR /ad_stack
RUN apt install -y python3-rosdep
RUN rosdep init && rosdep update
RUN git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
WORKDIR /ad_stack/AutowareAuto 
RUN vcs import < autoware.auto.$ROS_DISTRO.repos && rosdep install -y -i --from-paths src
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"


#### Install VNC server


#### INSTALLING lgsvl harness

## Install lgsvl_bridge

WORKDIR /ad_stack
RUN git clone https://github.com/lgsvl/ros2-lgsvl-bridge.git
WORKDIR /ad_stack/ros2-lgsvl-bridge
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; git checkout ${ROS_DISTRO}-devel; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

## Install lgsvl message types

WORKDIR /ad_stack
RUN git clone https://github.com/lgsvl/lgsvl_msgs.git
WORKDIR /ad_stack/lgsvl_msgs
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

## Copy sample parameters and package to stream detections to LGSVL format

COPY aw_object_detection/ /ad_stack/aw_object_detection

## Install the copied package
WORKDIR /ad_stack/aw_object_detection/packages/aw_lgsvl
# below sourcing of lgsvl messages probably not required for installation..
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; source /ad_stack/lgsvl_msgs/install/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" 
RUN pip3 install -r requirements.txt

# lgsvl_bridge
EXPOSE 9090/tcp 

COPY entrypoint.sh /
RUN ["chmod", "+x", "/entrypoint.sh"]
ENTRYPOINT /entrypoint.sh
