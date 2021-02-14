#!/bin/bash

date
sleep 10
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
      source stop_docker.bash
      sleep 1
      source start_docker.bash
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

