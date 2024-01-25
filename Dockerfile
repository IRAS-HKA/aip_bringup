##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV LC_ALL=C

RUN apt-get update && apt-get install --no-install-recommends -y \
    bash nano htop git sudo wget curl gedit pip && \
    rm -rf /var/lib/apt/lists/*

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=robot
ARG PASSWORD=robot
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=0
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /etc/bash.bashrc
RUN echo "export _colcon_cd_root=~/ros2_install" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

USER $USER 
RUN rosdep update

##############################################################################
##                                 General Dependencies                     ##
##############################################################################
USER root
RUN DEBIAN_FRONTEND=noninteractive apt update && apt install -y \
    gdb \
    ros-$ROS_DISTRO-moveit  \
    ros-$ROS_DISTRO-moveit-common  \
    ros-$ROS_DISTRO-moveit-servo  \
    ros-$ROS_DISTRO-xacro  \
    ros-$ROS_DISTRO-joint-trajectory-controller  \
    ros-$ROS_DISTRO-joint-state-broadcaster  \
    ros-$ROS_DISTRO-joint-state-publisher  \
    ros-$ROS_DISTRO-joint-state-publisher-gui  \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-sensor-msgs-py  \
    ros-$ROS_DISTRO-joy*  \
    ros-$ROS_DISTRO-cv-bridge

USER $USER
RUN pip install numpy scipy

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
RUN mkdir -p /home/"$USER"/dependencies_ws/src

# Following are clones as submodules (originally with git clone from iras Gitlab)
# RUN git clone -b driver-humble https://github.com/IRAS-HKA/kuka_eki.git
COPY ./dependencies/moveit_wrapper  /home/"$USER"/dependencies_ws/src/moveit_wrapper
COPY ./dependencies/iras_interfaces /home/"$USER"/dependencies_ws/src/iras_interfaces
COPY ./dependencies/kuka_eki  /home/"$USER"/dependencies_ws/src/kuka_eki
COPY ./dependencies/aip_cell_description /home/"$USER"/dependencies_ws/src/aip_cell_description

# Not necessary for aip
# COPY ./dependencies/ready2_educate /home/"$USER"/dependencies_ws/src/ready2_educate

# Still necessary? (no submodule, only local files in repo)
COPY ./dependencies/ros_environment /home/"$USER"/dependencies_ws/src/ros_environment
COPY ./dependencies/manipulation_tasks /home/"$USER"/dependencies_ws/src/manipulation_tasks
# RUN cd /home/"$USER"/dependencies_ws/manipulation_tasks/manipulation_tasks    && pip install numpy scipy


RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd /home/"$USER"/dependencies_ws && colcon build
RUN echo "source /home/$USER/dependencies_ws/install/setup.bash" >> /home/"$USER"/.bashrc

# RUN mkdir -p /home/"$USER"/dependencies
# COPY ./dependencies/manipulation_tasks /home/"$USER"/dependencies/manipulation_tasks
# RUN cd /home/"$USER"/dependencies/manipulation_tasks/manipulation_tasks    && pip install numpy scipy
# pip install manipulation_tasks


##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
USER $USER 
RUN mkdir -p /home/"$USER"/ros_ws/src
RUN cd /home/"$USER"/ros_ws && colcon build
RUN echo "source /home/$USER/ros_ws/install/setup.bash" >> /home/$USER/.bashrc
RUN mkdir -p /home/"$USER"/ros_ws/install/r2e_demos

WORKDIR /home/$USER/ros_ws

CMD /bin/bash