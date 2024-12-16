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
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV LC_ALL=C

RUN apt-get update && apt-get install --no-install-recommends -y \
    bash nano htop git sudo wget curl gedit pip \
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
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-forward-command-controller \
    libnlopt-cxx-dev \
    libnlopt-dev \
    ros-$ROS_DISTRO-graph-msgs \
    ros-$ROS_DISTRO-rviz-visual-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip install numpy scipy

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=robot
ARG PASSWORD=robot
ARG UID=1000
ARG GID=1000
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

# Set ROS2 DDS profile
COPY ./src/aip_bringup/config/dds_profile.xml /home/$USER
RUN chown $USER:$USER /home/$USER/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/dds_profile.xml

##############################################################################
##                                 dependecies_ws                           ##
##############################################################################
USER $USER 
RUN mkdir -p /home/$USER/dependencies_ws/src
WORKDIR /home/$USER/dependencies_ws/src

# Following are clones as submodules (originally with git clone from iras Gitlab)
# RUN git clone -b driver-humble https://github.com/IRAS-HKA/kuka_eki.git
COPY ./dependencies/moveit_wrapper  ./moveit_wrapper
COPY ./dependencies/aip_interfaces ./aip_interfaces
COPY ./dependencies/kuka_eki  ./kuka_eki
COPY ./dependencies/aip_cell_description ./aip_cell_description
COPY ./dependencies/trac_ik ./trac_ik
COPY ./dependencies/trac_ik_kinematics_plugin ./trac_ik_kinematics_plugin
COPY ./dependencies/trac_ik_lib ./trac_ik_lib

# object_detector_tensorflow_interfaces
RUN git clone -b humble https://github.com/eshan-savla/object_detector_tensorflow.git
RUN mv ./object_detector_tensorflow/ros/object_detector_tensorflow_interfaces . && \
    rm -rf ./object_detector_tensorflow

# Packing Planning Interfaces
RUN git clone --branch visualization https://github.com/SchmittAndreas/aip_packing_algorithm.git
RUN mv ./aip_packing_algorithm/aip_packing_planning_interfaces . && \
    rm -rf ./aip_packing_algorithm

# Grasp Planning Interfaces
RUN git clone https://github.com/LeoSc4/aip_grasp_planning.git
RUN mv ./aip_grasp_planning/aip_grasp_planning_interfaces . && \
    rm -rf ./aip_grasp_planning

# Stomp MoveIt (not necessary with Trac_IK?)
# RUN git clone https://github.com/henningkayser/stomp_moveit
# RUN vcs import < stomp_moveit/stomp_moveit.repos

# Still necessary? (no submodule, only local files in repo)
COPY ./dependencies/ros_environment ./ros_environment
COPY ./dependencies/manipulation_tasks ./manipulation_tasks
# RUN cd /home/"$USER"/dependencies_ws/manipulation_tasks/manipulation_tasks    && pip install numpy scipy

WORKDIR /home/$USER/dependencies_ws
RUN rosdep update --rosdistro $ROS_DISTRO
USER root
# RUN apt-get update 
RUN rosdep install --from-paths src --ignore-src -r -y
# RUN rm -rf /var/lib/apt/lists/*
USER $USER
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
RUN echo "source /home/$USER/dependencies_ws/install/setup.bash" >> /home/$USER/.bashrc

# RUN mkdir -p /home/"$USER"/dependencies
# COPY ./dependencies/manipulation_tasks /home/"$USER"/dependencies/manipulation_tasks
# RUN cd /home/"$USER"/dependencies/manipulation_tasks/manipulation_tasks    && pip install numpy scipy
# pip install manipulation_tasks

##############################################################################
##                                 ros_ws                                   ##
##############################################################################
RUN mkdir -p /home/$USER/ros_ws/src
WORKDIR /home/$USER/ros_ws

COPY ./src/aip_bringup ./src/aip_bringup

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
RUN echo "source /home/$USER/ros_ws/install/setup.bash" >> /home/$USER/.bashrc

##############################################################################
##                                 autostart                                ##
##############################################################################
RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros_ws/install/setup.bash"' \
    /ros_entrypoint.sh

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/dependencies_ws/install/setup.bash"' \
    /ros_entrypoint.sh

CMD ["bash"]