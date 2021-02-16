# Bringup #

## Install ##

Install ```tmux```  and ```nginx```

    sudo apt install tmux nginx


Link this folder from a folder accessible from a web server

    cd /var/www/html
    ln -s $MARRTINO_APPS_HOME/bringup .


Note: for ```shutdown``` you need to modify the sudo settings

    sudo visudo

add a line like this

    ubuntu ALL=(ALL) NOPASSWD: /sbin/poweroff, /sbin/reboot, /sbin/shutdown



## Run

* Configure and run a web server

        sudo service nginx start

* Run the server

        python wsbringup.py

    You can check command execution with ```tmux a -t bringup```

* Open a browser at the URL

        http://<ROBOT_IP>/bringup

* Connect to the robot and control its running modules


## Start server at boot

Use script ```1-bringup.bash``` at init

## Bringup servers

Run the bringup server

        python robot_bringup.py -server_port 9236
        python vision_bringup.py -server_port 9237
        python nav_bringup.py -server_port 9238
        python speech_bringup.py -server_port 9239

Send commands to bringup servers

        echo '@robot' | netcat -w 1 localhost 9236
        echo '@robotkill' | netcat -w 1 localhost 9236

        echo '@usbcam' | netcat -w 1 localhost 9237
        echo '@usbcamkill' | netcat -w 1 localhost 9237

        echo '@hokuyo' | netcat -w 1 localhost 9238
        echo '@rplidar' | netcat -w 1 localhost 9238
        echo '@loc' | netcat -w 1 localhost 9238
        echo '@movebase' | netcat -w 1 localhost 9238

        echo '@audio' | netcat -w 1 localhost 9239
        echo '@audiokill' | netcat -w 1 localhost 9239

