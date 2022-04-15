#!/bin/bash

IMAGENAME=iocchi/marrtino:base

# change setings here if needed


echo "Running image $IMAGENAME ..."

docker run -it \
    --name marrtinoclient --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    -v $MARRTINO_PLAYGROUND:/home/robot/playground \
    $IMAGENAME \
    tmux



