#!/bin/bash

IMAGENAME=iocchi/marrtino:base

# change setings here if needed


echo "Running image $IMAGENAME ..."

docker run -it \
    --name marrtinoclient --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    -e DISPLAY=$DISPLAY \
    -e ROS_IP=$ROS_IP \
    -e ROS_MASTER_URI=http://$ROBOT_IP:11311 \
    --privileged \
    --net=host \
    -v $MARRTINO_PLAYGROUND:/home/robot/playground \
    -v $MARRTINO_APPS_HOME:/home/robot/src/marrtino_apps \
    $IMAGENAME \
    tmux

