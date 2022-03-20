# Docker images

## Pre requisites

To use dockerized version of marrtino_apps you need on your OS

* docker (tested on v. 19.03)
* docker-compose (tested on v. 1.28.2)
* python

Set the environment variable `MARRTINO_APPS_HOME`to the folder where you downloaded your repository.

## Images available

* orazio
* base
* teleop
* navigation
* vision
* speech

## Configuration

Copy and edit `system_config.yaml`

        cd ~
        cp <...>/marrtino_apps/docker/system_config_template.yaml system_config.yaml
        nano system_config.yaml

            system:
              nginx: off

            simulator:
              stage: off

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

        cd <...>/marrtino_apps/docker
        ./system_update.bash

## Run

        cd <...>/marrtino_apps/docker
        ./start_docker.bash

## Quit

        cd <...>/marrtino_apps/docker
        ./stop_docker.bash


## Bringup servers

To interact with docker containers, see 
[bringup/README](https://bitbucket.org/iocchi/marrtino_apps/src/master/bringup/README.md)

## Docker access

        docker exec -it <container_name> tmux a

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

1) download  ```marrtino_social``` repository (https://github.com/artigianitecnologici/marrtino_social) and set environment variable ```MARRTINO_SOCIAL``` to the ```marrtino_social``` folder

2) enable social functionlity in ```system_config.yaml```

        social: on

3) start docker containers

        ./start_docker.bash

