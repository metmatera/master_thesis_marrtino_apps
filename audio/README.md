# MARRtino AudioServer program #

MARRtino Python AudioServer program to play sounds saved as uncompressed PCM 16 bit WAV files.

## Install ##

Requires pyglet
```
$ sudo -H pip install pyglet
```



## Run ##

* Run the server

```
$ python AudioServer [PORT]

```

* Connect with a TCP client sending the name of the sound to play (i.e., filename without ".wav")

* Notes:

- WAV files must be placed in the same directory of the server.

- Suggestion: to generate uncompressed PCM 16 bit WAV files, use audacity.


