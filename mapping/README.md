# Navigation #


## Install ##

* http://wiki.ros.org/gmapping


## Run ##

1. Run robot with laser 

2. Run mapper

```
roslaunch gmapping.launch
```

3. Drive the robot around the environment


4. Save the map

```
rosrun map_server map_saver -f my_map
```


