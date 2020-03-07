# laser #

Laser launchers (Hokuyo, RPlidar, RGBD depth2laser)

## Install ##

Depth2laser:

* https://github.com/ros-drivers/urg_node.git
* https://github.com/Slamtec/rplidar_ros.git
* https://bitbucket.org/ggrisetti/depth2laser


## Configure ##

Adjust file `config/transforms.txt` folder for 
camera and virtual laser transforms


## Run ##

Launch laser or rgbdlaser nodes

    roslaunch <laser>.launch

View RGBD laser

    rosrun rviz rviz -d rgbd_laser.rviz


