# xtion #

Applications using xtion sensor

## Install ##

Download the following packages in your catkin workspace and compile them with catkin_make:

* https://bitbucket.org/ggrisetti/depth2laser


## Run ##

* Run only the xtion node

```
$ roslaunch xtion.launch
```

* Run xtion and depth2laser

Note: check file transforms.txt in depth2laser/config folder for 
camera and virtual laser transforms

```
$ roslaunch xtion_laser.launch
$ rosrun rviz rviz -d xtion_laser.rviz
```

Use also the RGB channel in xtion_laser

```
$ roslaunch xtion_laser.launch rgb_mode:=9
```


