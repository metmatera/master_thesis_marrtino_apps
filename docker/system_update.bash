#!/bin/bash

echo "System update started"

source $MARRTINO_APPS_HOME/docker/stop_docker.bash 

sleep 5

cd $MARRTINO_APPS_HOME/docker
git pull
python3 dockerconfig.py
docker-compose pull
docker build -t marrtino:system -f Dockerfile.system .
docker-compose build
cd -

if [ -f /tmp/marrtinosocialon ] && [ "$MARRTINO_SOCIAL" != "" ]; then
  
  cd $MARRTINO_SOCIAL/docker
  docker-compose pull
  ./build_docker.bash
  cd -

fi

docker container prune -f
docker image prune -f

date > ~/log/last_systemupdate

source $MARRTINO_APPS_HOME/docker/start_docker.bash

echo "System update completed"
