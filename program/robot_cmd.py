#!/usr/bin/env python

import time
import os
import socket

from ctypes import cdll
lib = cdll.LoadLibrary('librobot_program.so')

AUDIO_SERVER_IP = '127.0.0.1'
AUDIO_SERVER_PORT = 9001
assock = None

userobot = True

# Begin/end

def begin():
	global assock
	print 'begin'

	if (userobot):
		print "Robot enabled"
		lib.start_robot_thread()
		lib.waitfor_connected()

	assock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	try:
		assock.connect((AUDIO_SERVER_IP, AUDIO_SERVER_PORT))
	except:
		print "Cannot connect to audio server %s:%d" %(AUDIO_SERVER_IP, AUDIO_SERVER_PORT)


def end():
	global assock
	print 'end'
	lib.stop()
	assock.close()
	assock=None
	time.sleep(0.5) # make sure stuff ends


# Robot motion

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


# Wait

def wait(r=1):
	print 'wait',r
	for i in range(0,r):
		time.sleep(3)


# Sounds

def bip(r=1):
	global assock
	for i in range(0,r):
		print 'bip'
		try:
			assock.send('bip')
		except:
			pass
		time.sleep(0.5)


def bop(r=1):
	global assock
	for i in range(0,r):
		print 'bop'
		try:
			assock.send('bop')
		except:
			pass
		time.sleep(0.5)



	


