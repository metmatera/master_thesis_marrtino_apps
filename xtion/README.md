# xtion #

Applications using xtion/astra sensor

## Install ##

Download the following packages in your catkin workspace and compile them with catkin_make:

Xtion:

* https://bitbucket.org/ggrisetti/thin_drivers

Astra:

* https://github.com/tfoote/ros_astra_camera.git

Depth2laser:

* https://bitbucket.org/ggrisetti/depth2laser


## Run ##

* Run only the xtion/astra node

```
$ roslaunch xtion.launch
```

```
$ roslaunch astra.launch
```

* Run xtion/astra and depth2laser

Note: check file transforms.txt in depth2laser/config folder for 
camera and virtual laser transforms

```
$ roslaunch xtion_laser.launch
$ rosrun rviz rviz -d xtion_laser.rviz
```

```
$ roslaunch astra_laser.launch
$ rosrun rviz rviz -d xtion_laser.rviz
```

Use also the RGB channel in xtion_laser

```
$ roslaunch xtion_laser.launch rgb_mode:=9
```


