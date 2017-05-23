# xtion #

Applications using xtion sensor

* Run only the xtion node

```
$ roslaunch xtion.launch
```

* Run xtion and depth2laser

- transforms
- xtion
- depth2laser

```
$ roslaunch xtion_laser.launch
$ rosrun rviz rviz -d xtion_laser.rviz
```
