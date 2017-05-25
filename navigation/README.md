# Navigation #


## Install ##

Download the following packages in your catkin workspace and compile them with catkin_make:

* https://bitbucket.org/iocchi/apriltags_ros

gradient_based_navigation


## Run ##

* To launch only the obstacle avoidance behavior

```
$ roslaunch obstacle_avoidance.launch
```

Note: controllers must publish on /desired_cmd_vel instead of /cmd_vel


