#!/bin/bash

SESSION=compose

# check if session already exists
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
  # Set up your session
  tmux -2 new-session -d -s $SESSION
fi

tmux new-window -t $SESSION:1 -n 'compose down'

tmux send-keys -t $SESSION:1 "cd \$MARRTINO_APPS_HOME/docker && docker-compose down" C-m

tmux send-keys -t $SESSION:1 "docker container prune -f" C-m

