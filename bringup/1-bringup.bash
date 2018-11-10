#!/bin/bash
date
#source /home/ubuntu/.bashrc
source /home/ubuntu/ros/catkin_ws/devel/setup.bash
export MARRTINO_APPS_HOME=/home/ubuntu/src/marrtino_apps
export DISPLAY=:0
cd /home/ubuntu/src/marrtino_apps/bringup
python wsbringup.py &
cd /home/ubuntu/src/marrtino_apps/config
python wsconfig.py &

