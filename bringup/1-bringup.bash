#!/bin/bash
date
source /home/ubuntu/ros/catkin_ws/devel/setup.bash
source /home/ubuntu/.bashrc
export MARRTINO_APPS_HOME=/home/ubuntu/src/marrtino_apps
cd /home/ubuntu/src/marrtino_apps/bringup
python wsbringup.py &
cd /home/ubuntu/src/marrtino_apps/config
python wsconfig.py &


