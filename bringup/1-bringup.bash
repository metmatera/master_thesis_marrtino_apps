#!/bin/bash
date

export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps
export MODIM_HOME=$HOME/src/modim

if [ ! "$1" == "-docker" ]; then
  echo "Running inside docker container..."
#  sudo service nginx start
#  sudo service shellinabox start

  echo "IP addresses: `hostname -I`"
  echo "docker exec -it <container name>  tmux   for shell access"
  #while [ ! -f "/tmp/quitrequest" ]; do
  #  sleep 5
  #done
fi

#source $HOME/.bashrc
source $HOME/ros/catkin_ws/devel/setup.bash
#export DISPLAY=:0
#export ROBOT_TYPE=marrtino

mkdir -p $HOME/log
cd $MARRTINO_APPS_HOME/bringup
python wsbringup.py &> $HOME/log/wsbringup.log &
cd $MARRTINO_APPS_HOME/config
python wsconfig.py &> $HOME/log/wsconfig.log &
roscore



