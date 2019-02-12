# camera #

Camera launchers: xtion/astra sensor

## Install ##

Download the following packages in your catkin workspace and compile them with catkin_make:

Xtion:

* https://bitbucket.org/ggrisetti/thin_drivers

Astra:

* https://github.com/tfoote/ros_astra_camera.git



## Run ##

* Run xtion/astra node

Launch

    roslaunch xtion.launch

or

    roslaunch astra.launch


To visualize images and depth data

    rosrun rviz rviz -d config/rgbd.rviz


