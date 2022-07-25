# Docker images

## Pre requisites

* Install `python` and `tmux`

        sudo apt install python tmux python3-yaml


* Install [docker](http://www.docker.com) (tested on v. 19.03, 20.10) 

    See also 
    [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
    In particular, add your user to the `docker` group and log out and in again, before proceeding.


* Install [docker-compose](https://docs.docker.com/compose/install/) (tested on v. 1.28.2)


* Set environment variable `MARRTINO_APPS_HOME` to the folder where you downloaded this repository.

    Example:

        export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps

    Add this command in `~/.bashrc` to make it permanent


* Set environment variables `ROS_IP`, `ROBOT_TYPE` according to your setup.

    Example for local setup with simulator:

        export ROS_IP=127.0.0.1
        export ROBOT_TYPE=stage

    Add these commands in `~/.bashrc` to make them permanent

* Set user and group ID

    Add these commands in `~/.bashrc` to make them permanent

        export UID=$(id -u)
        export GID=$(id -g)


* Set X11 server to accept clients (from docker containers)

        xhost +


## Images available

* orazio
* base
* teleop
* navigation
* vision
* speech

Build all images

    ./docker_build.bash

Note: in some cases, it may be needed to recompile the orazio ROS nodes

    docker exec -it orazio tmux a
    cd ~/ros/catkin_ws
    catkin-make clean
    catkin-make

and commit the docker image.



## Configuration

Copy and edit `system_config.yaml`

        cd $MARRTINO_APPS_HOME
        cp docker/system_config_template.yaml system_config.yaml
        nano system_config.yaml

            system:
              nginx: off

            simulator:
              stage: off   # off|on|x11|vnc

            robot:
              motorboard: off  # arduino|ln298|pka03|marrtino2019
              4wd: off
              joystick: off
              laser: off
              camera: off

            functions:
              navigation: off
              vision: off
              speech: off
              mapping: off
              social: off



## Update and build

        cd $MARRTINO_APPS_HOME/docker
        ./system_update.bash

## Run

        cd $MARRTINO_APPS_HOME/docker
        ./start_docker.bash

    Note: for VNC option in stage, GUI is accessible through a browser at `http://localhost:3000`

## Quit

        cd $MARRTINO_APPS_HOME/docker
        ./stop_docker.bash


## Bringup servers

To interact with docker containers, see 
[bringup/README](https://bitbucket.org/iocchi/marrtino_apps/src/master/bringup/README.md)

## Docker access

        docker exec -it <container_name> tmux a

## Autostart


To start devices and functionalities at boot automatically, see
[start/README](https://bitbucket.org/iocchi/marrtino_apps/src/master/start/README.md)

## Docker push

    Edit `Dockerfile.<component>` setting last docker build date

        RUN echo "<date>" > /tmp/lastdockerbuild


    Commit ang push last changes

        git commit -am "dockerhub <date>"
        git push



    Push on Docker hub (you may need to change the docker tags)

        ./docker_build.bash
        ./docker_push.bash


## MARRtino social

To use MARRtino social functionalities:

1) download  [`marrtino_social` repository](https://github.com/artigianitecnologici/marrtino_social) and set environment variable `MARRTINO_SOCIAL` to the folder

    Example:

        export MARRTINO_SOCIAL=$HOME/src/marrtino_social


2) enable social functionlity in `system_config.yaml`

        social: on

3) start docker containers

        ./start_docker.bash


## Issues

* Error when stopping the containers

    If you get errors in stopping the containers, try the following command and then stop che contaimners again.

        sudo aa-remove-unknown


