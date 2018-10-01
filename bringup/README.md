# Bringup #

## Install ##

Link this folder from a folder accessible from a web server

    cd /var/www/html
    ln -s $MARRTINO_APPS_HOME/bringup .


Note: for ```shutdown``` you need to modify the sudo settings

    sudo visudo

add a line like this

    ubuntu ALL=(ALL) NOPASSWD: /sbin/poweroff, /sbin/reboot, /sbin/shutdown


## Run ##

* Configure and run a web server

    sudo service nginx start

* Open a browser at the URL

    http://<ROBOT_IP>/bringup


* Connect to the robot and run commands


