#!/usr/bin/env python

import time
import pyttsx
import os


from ctypes import cdll
lib = cdll.LoadLibrary('librobot_program.so')

tts_engine = None
userobot = True


def begin():
	global tts_engine
	print 'begin'

	if (userobot):
		print "Robot enabled"
		lib.start_robot_thread()
		lib.waitfor_connected()

	tts_engine = pyttsx.init()
	voices = tts_engine.setProperty('voice','italian')
	tts_engine.setProperty('rate', 150)


def end():
	print 'end'
	lib.stop()

	time.sleep(0.5) # make sure stuff ends

def stop():
	print 'stop'
	lib.stop()


def forward(r=1):
	print 'forward',r
	lib.forward(r)


def backward(r=1):
	print 'backward',r
	lib.backward(r)


def left(r=1):
	print 'left',r
	lib.left(r)


def right(r=1):
	print 'right',r
	lib.right(r)


def wait(r=1):
	print 'wait',r
	for i in range(0,r):
		time.sleep(3)


# generate wav with:
# pico2wave -l "it-IT" -w start.wav "Bene! Si Parte!"

# Run this command in background to keep the audio drivers synchronized
# play --no-show-progress --null -c2 synth sin gain -100 


def hello():
	global tts_engine
	print 'hello'
	os.system("aplay -Dhw:0,0 ~/Music/ciao.wav")
	#tts_engine.say('Ciao! Sono un robot parlante. Aspetto i tuoi comandi.')
	#tts_engine.runAndWait()
	time.sleep(1)


def start():
	global tts_engine
	print 'start'
	os.system("aplay -Dhw:0,0 ~/Music/start.wav")
	#tts_engine.say('Bene! Si parte.')
	#tts_engine.runAndWait()
	time.sleep(1)


# sox -n --no-show-progress -G --channels 1 -r 16000 -b 16 -t wav bip.wav synth 0.2 sine 800 
# sox -n --no-show-progress -G --channels 1 -r 16000 -b 16 -t wav bop.wav synth 0.25 sine 400 


def bip(r=1):
	for i in range(0,r):
		print 'bip'
		os.system("aplay ~/Music/bip.wav")
		time.sleep(0.5)


def bop(r=1):
	for i in range(0,r):
		print 'bop'
		os.system("aplay ~/Music/bop.wav")
		#os.system("aplay -Dhw:0,0 ~/Music/bop.wav")
		time.sleep(0.5)


