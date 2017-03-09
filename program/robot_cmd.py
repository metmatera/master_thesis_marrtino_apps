#!/usr/bin/env python

import time
import pyttsx
import os


from ctypes import cdll
lib = cdll.LoadLibrary('librobot_program.so')

tts_enging = None

def begin():
	global tts_engine
	print 'begin'
	lib.start_robot_thread()
	lib.waitfor_connected()
	tts_engine = pyttsx.init()
	voices = tts_engine.setProperty('voice','italian')
	tts_engine.setProperty('rate', 120)


def end():
	print 'end'
	lib.stop()

	time.sleep(0.5) # make sure stuff ends

def stop():
	print 'stop'
	lib.stop()


def forward(r=1):
	for i in range(0,r):
		print 'forward'
		lib.forward()


def backward(r=1):
	for i in range(0,r):
		print 'backward'
		lib.backward()


def left(r=1):
	for i in range(0,r):
		print 'left'
		lib.left()


def right(r=1):
	for i in range(0,r):
		print 'right'
		lib.right()


def wait(r=1):
	for i in range(0,r):
		print 'wait'
		time.sleep(3)


def hello():
	global tts_engine
	print 'hello'
	tts_engine.say('Ciao! Io sono un robot parlante.')
	tts_engine.runAndWait()


def bip(r=1):
	for i in range(0,r):
		print 'bip'
		os.system("play --no-show-progress --null --channels 1 synth 0.2 sine 800")


def bop(r=1):
	for i in range(0,r):
		print 'bop'
		os.system("play --no-show-progress --null --channels 1 synth 0.25 sine 400")

