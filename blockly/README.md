# MARRtino Blockly program #

MARRtino Blockly programming.

## Install ##

Follow installation instructions in MARRtino Python program.

* Blockly

Blockly code is installed as a git submodule. Install with

```
git submodule init
git submodule update

```

* Websocket 

Install tornado websocket library

```
sudo -H easy_install tornado
```


## Blocky programming ##


* Run a robot

Simulator

```
$ cd marrtino_apps/stage
$ roslaunch simrobot.launch 
```

Real robot

```
$ cd marrtino_apps/robot
$ roslaunch robot.launch 
```



* Run the websocket server (on the robot machine)

```
$ cd marrtino_apps/blockly
$ python websocket_robot.py
```

* Run the Blockly Robot app (on the client machine)

```
$ cd marrtino_apps/blockly
$ firefox blockly_robot.html
```

Set IP of the robot (i.e., IP of the machine running the websocket server) and connect to it.

Use Blockly to build a program and run it through the ```Run``` button.

Use the ```Stop``` button at any time to stop the program and the robot.

Use the ```Export/Import block code``` buttons to save and load blockly code in XML format. Copy and paste xml code on files to store them permanently.

----

## Develop new blocks ##

https://blockly-demo.appspot.com/static/demos/blockfactory/index.html




