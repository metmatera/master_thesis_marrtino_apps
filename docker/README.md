# Docker images

## Images available

* orazio
* base
* teleop
* navigation

other soon...

## Profiles

        simulator       stage simulator (no robot devices)
        robot           robot device (amd64 arch)
        robotarm64      robot device (arm64 arch)
        robot2018       robot device (amd64 arch, ArduinoMB 2018)
        robot2018arm64  robot device (arm64 arch, ArduinoMB 2018)

Export your system profile in `$HOME/.system_profile`

        echo "simulator" > $HOME/.system_profile


## Update and build

        cd <...>/marrtino_apps/docker
        ./system_update.bash

## Run

        cd <...>/marrtino_apps/docker
        ./start_docker.bash

## Quit

        cd <...>/marrtino_apps/docker
        ./stop_docker.bash


## orazio

From the container:

* flash firmware (connect Arduino with USB)

        cd ~/src/srrg/srrg2_orazio/srrg2_orazio/firmware_build/atmega2560/
        make
        make orazio.hex

* run orazio web server

        cd ~/src/marrtino_apps/config
        ./run_orazio2_web.bash

* launch robot node

        cd ~/src/marrtino_apps/robot
        roslaunch robot.launch


## orazio (version 2018)

* build

        docker build -t orazio:2018 -f Dockerfile.orazio2018 .

* run

        docker run -it --privileged -v /dev:/dev --net host orazio:2018 tmux

