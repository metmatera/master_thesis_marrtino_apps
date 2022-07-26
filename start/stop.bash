#!/bin/bash

SESSION=compose

tmux send-keys -t $SESSION:4 "cd \$MARRTINO_APPS_HOME/start" C-m
tmux send-keys -t $SESSION:4 "python3 autostart.py --kill" C-m


