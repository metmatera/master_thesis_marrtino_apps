# Using stage simulator for MARRtino #

* Download stage_environments package in your ROS catkin workspace and compile it

```
https://bitbucket.org/iocchi/stage_environments.git
```



## simrobot.launch ##

Launch the simulator

```
$ roslaunch simrobot.launch
```

Topics services and frames used by the simulator

```
Node [/stageros]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /clock [rosgraph_msgs/Clock]
 * /odom [nav_msgs/Odometry]
 * /base_pose_ground_truth [nav_msgs/Odometry]
 * /scan [sensor_msgs/LaserScan]

Subscriptions: 
 * /stageGUIRequest [unknown type]
 * /clock [rosgraph_msgs/Clock]
 * /cmd_vel [unknown type]

Services: 
 * /stageros/set_logger_level
 * /stageros/get_loggers

Frames:
- base_footprint_frame 
- base_frame
- laser_frame
```


## Testing ##

Use teleop app to drive the robot in the environment.



