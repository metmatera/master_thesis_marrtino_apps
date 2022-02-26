#!/bin/bash

# This script runs in marrtino user space

SESSION=compose

# check if session already exists
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
  # Set up your session
  tmux -2 new-session -d -s $SESSION
fi

tmux rename-window -t $SESSION:0 'compose up'

tmux send-keys -t $SESSION:0 "cd \$MARRTINO_APPS_HOME/docker" C-m

tmux send-keys -t $SESSION:0 "python3 dockerconfig.py " C-m
sleep 1 # needed to complete writing /tmp/* files

tmux send-keys -t $SESSION:0 "export ROBOT_TYPE=`cat /tmp/robottype` " C-m
tmux send-keys -t $SESSION:0 "export CAMRES='`cat /tmp/cameraresolution`'" C-m

tmux send-keys -t $SESSION:0 "docker-compose up" C-m


