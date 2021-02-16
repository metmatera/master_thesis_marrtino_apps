#!/bin/bash



SESSION=compose

tmux -2 new-session -d -s $SESSION

tmux rename-window -t $SESSION:0 'compose up'

tmux send-keys -t $SESSION:0 "cd \$MARRTINO_APPS_HOME/docker" C-m

tmux send-keys -t $SESSION:0 "python dockerconfig.py" C-m

tmux send-keys -t $SESSION:0 "docker-compose up" C-m


