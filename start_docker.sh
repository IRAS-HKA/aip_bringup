#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")

echo "Run Container"
docker run \
    --name aip_bringup \
    --privileged \
    -it \
    -e DISPLAY=$DISPLAY \
    -v $PWD/src:/home/robot/ros_ws/src:rw \
    -v $PWD/.vscode:/home/robot/dependencies_ws/src/.vscode \
    -v /dev:/dev  \
    --net host \
    --rm \
    --ipc host \
    iras/aip_bringup:humble
