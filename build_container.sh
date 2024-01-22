#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
# --no-cache 
docker build --build-arg UID="$uid" --build-arg GID="$gid" --build-arg ROS_DISTRO=humble --build-arg DOMAIN_ID=66 -t iras/aip_bringup:humble .

