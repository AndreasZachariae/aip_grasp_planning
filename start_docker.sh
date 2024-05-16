#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
       
xhost + local:root
docker run \
    --name aip_grasp_planning \
    --privileged \
    -it \
    -e DISPLAY=$DISPLAY \
    --env-file .env \
    -v $PWD/src:/home/robot/ros_ws/src:rw \
    -v $PWD/pcl_recordings:/home/robot/ros_ws/pcl_recordings:rw \
    -v $PWD/.vscode:/home/robot/dependencies_ws/src/.vscode \
    -v $PWD/dependencies:/home/robot/dependencies_ws/src \
    -v /dev:/dev  \
    --net host \
    --rm \
    --ipc host \
    iras/aip_grasp_planning:humble
# --env-file .env \ set environment including ROS_DOMAIN_ID