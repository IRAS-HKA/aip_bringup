#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
docker build --no-cache --build-arg UID="$uid" --build-arg GID="$gid" --build-arg ROS_DISTRO=humble --build-arg DOMAIN_ID=7 -t iras/aip_bringup:humble .

