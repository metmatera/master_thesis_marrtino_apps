# MARRtino Python program #

MARRtino Python programming with basic actions.

## Install ##

* Download arduino_robot https://bitbucket.org/ggrisetti/arduino_robot 
and compile it (preferred method catkin).

* Make sure the library 'librobot_program.so' is in your LD_LIBRAY_PATH

* Install libraries for speech and sound
```
$ sudo apt install sox
$ sudo pip install pyttsx
```
In case of 'locale.Error: unsupported locale setting' set 
```
export LC_ALL=C
```



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

== Client/Server ==

* Run the server

```
$ ./robot_program_server.py
```

* Sends the commands from a client. Commands separated by new lines or ;
(see robot_program_client.py for an example).



