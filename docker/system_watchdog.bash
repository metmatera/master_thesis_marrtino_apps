#!/bin/bash

date

rm -f ~/log/shutdownrequest ~/log/rebootrequest ~/log/systemupdate ~/log/dockerrestart ~/log/logincompleted


# add tmux session to .bashrc
grep logincompleted ~/.bashrc

if [ $? != 0 ]; then
  echo "# check if session already exists" >> ~/.bashrc
  echo "tmux has-session -t compose 2>/dev/null" >> ~/.bashrc
  echo "" >> ~/.bashrc
  echo "if [ $? != 0 ]; then" >> ~/.bashrc
  echo "  tmux -2 new-session -d -s compose" >> ~/.bashrc
  echo "  touch ~/log/logincompleted" >> ~/.bashrc
  echo "fi" >> ~/.bashrc
  echo "" >> ~/.bashrc
  touch ~/log/logincompleted
  sleep 10
fi

echo "Waiting for login ..."

# add `touch ~/log/logincompleted` at the end of .bashrc
while [ ! -f ~/log/logincompleted ]; do
    sleep 2.5
done
rm -f ~/log/logincompleted

echo "Starting docker ..."

cd /home/marrtino/bin && source start_docker.bash

sleep 30


while [ ! -f ~/log/shutdownrequest ] && [ ! -f  ~/log/rebootrequest ]; do

    sleep 5

    if [ -f  ~/log/systemupdate ]; then
      echo "System update..."
      source system_update.bash
      rm -f ~/log/systemupdate
    fi

    if [ -f  ~/log/dockerrestart ]; then
      echo "docker restart..."
      source restart_docker.bash
      rm -f ~/log/dockerrestart
    fi

done

if [ -f  ~/log/shutdownrequest ]; then
  rm -f ~/log/shutdownrequest
  echo "Shutdown..."
  sudo halt
fi

if [ -f  ~/log/rebootrequest ]; then
  rm -f ~/log/rebootrequest
  echo "Reboot..."
  sudo reboot
fi

