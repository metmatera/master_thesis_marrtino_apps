# camera #

Camera launchers: usbcam/xtion/astra sensor

## Install ##

Download the following packages in your catkin workspace and compile them with catkin_make:

Astra:

* https://github.com/tfoote/ros_astra_camera.git

Xtion:

* https://bitbucket.org/ggrisetti/thin_drivers



## Run ##

* Run usbcam/xtion/astra node

Launch

    roslaunch astra.launch

or

    roslaunch usbcam.launch

or

    roslaunch xtion.launch


To visualize images and depth data

    rosrun rviz rviz -d config/rgbd.rviz

or

    rosrun image_view image_view image:=/rgb/image_raw


Note: To visualize images from another PC, export ROS_MASTER_URI. This feature may be very slow specially if using a wireless network.

Note: usbcam.launch have default settings that works on Logitech C920 camera.
If these settings do not work for other cameras, change them as appropriate.


