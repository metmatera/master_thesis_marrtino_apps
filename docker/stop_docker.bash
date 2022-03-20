#!/bin/bash

SESSION=compose


tmux send-keys -t $SESSION:1 "cd \$MARRTINO_APPS_HOME/docker && docker-compose down" C-m


if [ -f /tmp/marrtinosocialon ] && [ "$MARRTINO_SOCIAL" != "" ]; then

  tmux send-keys -t $SESSION:3 "cd \$MARRTINO_SOCIAL/docker && docker-compose down" C-m

fi

tmux send-keys -t $SESSION:1 "docker container prune -f" C-m

