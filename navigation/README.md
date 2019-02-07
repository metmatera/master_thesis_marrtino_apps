# Navigation #


## Install ##

* ROS navigation packages

```
sudo apt-get install ros-kinetic-navigation
```


* ROS package: gradient_based_navigation

```
https://github.com/Imperoli/gradient_based_navigation
```


## Run ##

* Standard navigation

To launch navigation modules, use the script 

    python runnav.py

or start localizer and navigation modules manually

    roslaunch srrg_localizer.launch map_name:=<MAP> initial_pose_x:=<X> initial_pose_y:=<Y> initial_pose_a:=<THETA>
    roslaunch move_base.launch


To send target goals to the robot, use the script

    python move.py <GX> <GY> <GTheta DEG>

(GX,GY,GTheta): target pose in map coordinates

* To launch only the obstacle avoidance behavior


    roslaunch obstacle_avoidance.launch


Note: to use obstacle avoidance, 
controllers must publish on /desired_cmd_vel instead of /cmd_vel


