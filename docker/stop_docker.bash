#!/bin/bash

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

if [ "$MACHTYPE" == "aarch64" ] || [ "$MACHTYPE" == "armv7l" ]; then
  ARCH="arm64"
  if [ "$MBTYPE" = "ArduinoMotorShield" ]; then
    DOCKER_PROFILE="robot2018arm64"
  else
    DOCKER_PROFILE="robotarm64"
  fi
else
  ARCH=""
  if [ "$MBTYPE" = "ArduinoMotorShield" ]; then
    DOCKER_PROFILE="robot2018"
  else
    DOCKER_PROFILE="robot"
  fi
fi

SESSION=compose

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:1 -n 'compose down'

tmux send-keys -t $SESSION:1 "cd \$MARRTINO_APPS_HOME/docker && docker-compose --profile $DOCKER_PROFILE down" C-m

tmux send-keys -t $SESSION:1 "docker container prune -f" C-m

