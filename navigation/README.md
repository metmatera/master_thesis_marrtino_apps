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

* Standard navigation nodes: amcl, move_base

```
$ roslaunch move_base.launch
```


* To launch only the obstacle avoidance behavior

```
$ roslaunch obstacle_avoidance.launch
```

Note: controllers must publish on /desired_cmd_vel instead of /cmd_vel


