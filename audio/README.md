# MARRtino AudioServer program #

MARRtino Python AudioServer program to play sounds saved as uncompressed PCM 16 bit 44100 Hz WAV files.

## Install ##

* Requires pyglet

```
$ sudo -H pip install pyglet
```

* Libraries for speech and sound
```
$ sudo apt install sox
```
In case of 'locale.Error: unsupported locale setting' set 
```
export LC_ALL=C
```

* Synth generation

```
$ sox -n --no-show-progress -G --channels 1 -r 44100 -b 16 -t wav bip.wav synth 0.25 sine 800 
$ sox -n --no-show-progress -G --channels 1 -r 44100 -b 16 -t wav bop.wav synth 0.25 sine 400 
```

* Voice generation
```
$ pico2wave -l "it-IT" -w start.wav "Bene! Si Parte!"
```

* Notes:

- WAV files must be placed in the same directory of the server.

- Suggestion: to generate uncompressed PCM 16 bit 44100 Hz WAV files, use audacity.



## Run ##

* Run the server

```
$ python AudioServer [PORT]

```

Note: to run it from a remote shell set 'export DISPLAY=:0'

* Connect with a TCP client sending the name of the sound to play (i.e., filename without ".wav")


## Adding new sounds ##

* Record a sound and write the dile in the AudioServer.py directory (WAV, PCM 16 bit 44100 Hz)

* Send a string with the name of the file (without ".wav") from a TCP client


