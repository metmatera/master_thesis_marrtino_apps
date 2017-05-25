# MARRtino Python program #

MARRtino Python programming with basic actions.

## Install ##

Download the following packages in your catkin workspace and compile them with catkin_make:

* https://gitlab.com/srrg-software/srrg_cmake_modules
* https://gitlab.com/srrg-software/srrg_orazio_core
* https://bitbucket.org/iocchi/marrtino_programming


* Make sure the library 'librobot_program.so' is in your LD_LIBRAY_PATH
(if you use standard catkin build system, the library will be in <catkin_ws>/devel/lib)




## Programming ##

* Write and run your Python program (see robot_program_1.py as an example).

```
#!/usr/bin/env python

from robot_cmd import *

begin()

<your program>

end()
```


Available commands (implemented in robot_cmd.py):

```
begin()
end()
stop()
forward(r=1)
backward(r=1)
left(r=1)
right(r=1)
wait(r=1)
hello()
bip(r=1)
bop(r=1)
```

## Client/Server ##

* Run the server

```
$ ./robot_program_server.py <PORT>
```

* Send the commands from a client. Commands separated by new lines or ;.
Use robot_program_client.py for example.
```
$ ./robot_program_client.py <HOST> <PORT>
```

## Audio ##

* Run the AudioServer (see audio app) to play sounds

* To add new sounds, add a WAV file in the audio app directory (see the README file in audio app)
and add a command in robot_cmd.py (similarly to bip and bop functions).

## ROS ##

* Run a ROS node to control the robot

Real MARRtino
```
rosrun srrg_orazio_ros orazio_robot_node <parameters...>
```

Simulator
```
$ cd <..>/marrtino_apps/stage/
$ roslaunch simrobot.launch
```

* Write and run your Python program (see robot_program_ros_1.py as an example).

```
#!/usr/bin/env python

from robot_cmd_ros import *

begin()

<your program>

end()
```


