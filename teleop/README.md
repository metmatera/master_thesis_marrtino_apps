# Teleop #

## Install ##

Download the following packages in your catkin workspace and compile them with catkin_make:


    https://github.com/Imperoli/gradient_based_navigation


For joystick control, you also need the ROS node joy.


    sudo apt install ros-kinetic-joy


For obstacle avoidance, you also need a laser device (see [laser](https://bitbucket.org/iocchi/marrtino_apps/src/master/laser/) section)


## Run ##

* Joystick/keyboard control


        roslaunch teleop.launch [use_joystick:=false]


* Joystick/keyboard + real robot control


        roslaunch robot_teleop.launch  [use_joystick:=false]


* Keyboard/Joystick + real robot control with obstacle avoidance


        roslaunch robot_teleop_nav.launch [use_joystick:=false]


* Visualizing sensor data

    If you want to display from a remote PC

        export ROS_MASTER_URI=http://<ROBOT_IP>:11311/

    Run the rviz visualizer

        rosrun rviz rviz -d robot_nav.rviz


