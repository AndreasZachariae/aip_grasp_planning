#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")

    #--no-cache 
docker build \
    --build-arg UID="$uid" \
    --build-arg GID="$gid" \
    --build-arg CACHE_BUST="$(date +%s)" \
    --build-arg ROS_DISTRO=humble \
    -t iras/aip_grasp_planning:humble \
    .
