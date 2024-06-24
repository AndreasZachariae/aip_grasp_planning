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
    -v $PWD/pcl_recordings:/home/robot/ros_ws/pcl_recordings:rw \
    -v $PWD:/home/robot/ros_ws/src/aip_grasp_planning:rw \
    -v /dev:/dev  \
    --net host \
    --rm \
    --ipc host \
    iras/aip_grasp_planning:humble
# --env-file .env \ set environment including ROS_DOMAIN_ID
    # -v $PWD:/home/robot/ros_ws/src/aip_grasp_planning:rw \
