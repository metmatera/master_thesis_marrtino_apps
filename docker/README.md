# Docker images

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

## docker access

        docker exec -it <container_name> tmux a


