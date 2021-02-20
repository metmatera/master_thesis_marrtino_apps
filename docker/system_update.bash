#!/bin/bash

source $MARRTINO_APPS_HOME/docker/stop_docker.bash 

sleep 5

cd $MARRTINO_APPS_HOME/docker
git pull
python3 dockerconfig.py
docker-compose pull
docker-compose build
cd -

docker container prune -f
docker image prune -f

date > ~/log/last_systemupdate

source $MARRTINO_APPS_HOME/docker/start_docker.bash

