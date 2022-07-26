#!/bin/bash

# This script runs in marrtino user space

SESSION=compose

# check if session already exists
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then

  # Default value for DISPLAY
  if [ "$DISPLAY" == "" ]; then
    export DISPLAY=:0
  fi

  # Launch and set up session
  tmux -2 new-session -d -s $SESSION
  tmux rename-window -t $SESSION:0 'marrtino up'
  tmux new-window -t $SESSION:1 -n 'marrtino down'
  tmux new-window -t $SESSION:2 -n 'social up'
  tmux new-window -t $SESSION:3 -n 'social down'
  tmux new-window -t $SESSION:4 -n 'autostart'
fi

tmux send-keys -t $SESSION:4 "cd \$MARRTINO_APPS_HOME/start" C-m
tmux send-keys -t $SESSION:4 "python3 autostart.py " C-m


