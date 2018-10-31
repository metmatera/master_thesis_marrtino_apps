# Bringup #

## Install ##

Link this folder from a folder accessible from a web server

    cd /var/www/html
    ln -s $MARRTINO_APPS_HOME/bringup .


Note: for ```shutdown``` you need to modify the sudo settings

    sudo visudo

add a line like this

    ubuntu ALL=(ALL) NOPASSWD: /sbin/poweroff, /sbin/reboot, /sbin/shutdown

Install '''tmux'''

    sudo apt install tmux


## Run ##

* Configure and run a web server

        sudo service nginx start

* Run the server

        python wsbringup.py

    You can check command execution with '''tmux a'''

* Open a browser at the URL

        http://<ROBOT_IP>/bringup

* Connect to the robot and control its running modules


