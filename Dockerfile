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
COPY ./aip_grasp_planning/config/dds_profile.xml /home/$USER

RUN chown $USER:$USER /home/$USER/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/dds_profile.xml

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
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-perception-pcl\
    ros-$ROS_DISTRO-rviz2

USER $USER
RUN pip install numpy scipy open3d opencv-python

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
RUN mkdir -p /home/$USER/ros_ws/src
WORKDIR /home/$USER/ros_ws/src

ARG CACHE_BUST

# Grasp planning
RUN git clone --recurse-submodules https://github.com/LeoSc4/aip_grasp_planning.git


# ODTF Interfaces forked by AIP2
RUN git clone https://github.com/eshan-savla/object_detector_tensorflow.git
RUN mv ./object_detector_tensorflow/ros/object_detector_tensorflow_interfaces . && \
    rm -rf ./object_detector_tensorflow

# Packing Planning Interfaces
RUN git clone https://github.com/SchmittAndreas/aip_packing_algorithm.git
RUN mv ./aip_packing_algorithm/aip_packing_planning_interfaces . && \
    rm -rf ./aip_packing_algorithm

RUN mkdir -p /home/"$USER"/ros_ws/pcl_recordings

RUN chown -c $USER:$USER /home/"$USER"/ros_ws/pcl_recordings

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build \
--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
RUN echo "source /home/$USER/ros_ws/install/setup.bash" >> /home/$USER/.bashrc

WORKDIR /home/$USER/ros_ws

COPY --chown=$USER --chmod=0755 ./entrypoint.sh /home/$USER/ros_ws/entrypoint.sh
CMD ["/bin/bash"]

# ENTRYPOINT ["./entrypoint.sh"]


