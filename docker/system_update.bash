#!/bin/bash

source stop_docker.bash 

MACHTYPE=`uname -m`
MBTYPE=`cat $HOME/.marrtino_motorboard`
SYSTEM_PROFILE=`cat $HOME/.system_profile`

if [ "$SYSTEM_PROFILE" = "" ]; then
  echo "System profile not set. File $HOME/.system_profile is empty." 
  exit 1
fi

if [ "$MBTYPE" = "" ]; then
  echo "Motor board not set. File $HOME/.marrtino_motorboard is empty."
  MBTYPE="MARRtinoMB2019"
  echo "Using default value: $MBTYPE"
fi

echo "System update"
echo "- system arch: $MACHTYPE"
echo "- system profile: $SYSTEM_PROFILE"
echo "- motor board: $MBTYPE"

if [ "$MACHTYPE" == "aarch64" ] || [ "$MACHTYPE" == "armv7l" ]; then
  if [ "$MBTYPE" = "ArduinoMotorShield" ]; then
    ARCH="2018-arm64"
  else
    ARCH="arm64"
  fi
else
  if [ "$MBTYPE" = "ArduinoMotorShield" ]; then
    ARCH="2018"
  else
    ARCH=""
  fi
fi

docker pull iocchi/rchomeedu-1804-melodic:${ARCH}

cd $HOME/src/marrtino_apps && git pull
cd $HOME/src/rc-home-edu-learn-ros && git pull

if [ "$SYSTEM_PROFILE" = "simulator" ]; then
  docker pull iocchi/stage_environments:${ARCH}
  cd $HOME/src/stage_environments && git pull
fi

if [ "$SYSTEM_PROFILE" = "robot" ]; then
  docker pull iocchi/orazio:${ARCH}
fi

cd $MARRTINO_APPS_HOME/docker && docker-compose --profile $SYSTEM_PROFILE build

docker container prune -f
docker image prune -f

date > ~/log/last_systemupdate

source start_docker.bash

