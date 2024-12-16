#!/bin/sh

# Define the command to run inside the Docker container, default is bash
COMMAND=${1:-bash}

uid=$(eval "id -u")
gid=$(eval "id -g")

docker run \
    --name aip_bringup \
    --privileged \
    -it \
    -e DISPLAY=$DISPLAY \
    --env-file .env \
    -v $PWD/src:/home/robot/ros_ws/src:rw \
    -v $PWD/.vscode:/home/robot/ros_ws/src/.vscode \
    -v $PWD/dependencies:/home/robot/dependencies_ws/src \
    -v /dev:/dev  \
    --net host \
    --rm \
    --ipc host \
    iras/aip_bringup:humble \
    $COMMAND
