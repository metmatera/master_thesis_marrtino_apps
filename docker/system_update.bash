#!/bin/bash

source stop_docker.bash 

sleep 5

cd $MARRTINO_APPS_HOME/docker && git pull && python3 dockerconfig.py && docker-compose build

docker container prune -f
docker image prune -f

date > ~/log/last_systemupdate

source start_docker.bash

