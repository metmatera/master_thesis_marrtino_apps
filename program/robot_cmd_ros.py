#!/usr/bin/env python

import time
import os
import socket
import math

import rospy
from geometry_msgs.msg import Twist

cmd_pub = None

AUDIO_SERVER_IP = '127.0.0.1'
AUDIO_SERVER_PORT = 9001
assock = None

userobot = True


# Good values
tv_good = 0.2
rv_good = 0.8

move_step = 0.5;



# Begin/end

def begin():
	global assock
	global cmd_pub
	print 'begin'

	if (userobot):
		print "Robot enabled"
		rospy.init_node('robot_cmd')
		cmd_pub = rospy.Publisher('cmd_vel', Twist)

	assock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	try:
		assock.connect((AUDIO_SERVER_IP, AUDIO_SERVER_PORT))
	except:
		print "Cannot connect to audio server %s:%d" %(AUDIO_SERVER_IP, AUDIO_SERVER_PORT)


def end():
	global assock
	print 'end'
	#lib.stop()
	assock.close()
	assock=None
	time.sleep(0.5) # make sure stuff ends


# Robot motion

def stop():
	print 'stop'
	#lib.stop()


def move(lx,az,tm):
	global cmd_pub
	delay = 0.1 # sec
	rate = rospy.Rate(1/delay) # Hz
	cnt = 0.0
	msg = Twist()
	msg.linear.x = lx
	msg.angular.z = az
	msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y =  0
	while not rospy.is_shutdown() and cnt<tm:		
		cmd_pub.publish(msg)
		cnt = cnt + delay
		rate.sleep()

def forward(r=1):
	global tv_good
	print 'forward',r
	move(tv_good,0.0,r*move_step/tv_good)
	

def backward(r=1):
	print 'backward',r
	move(-tv_good,0.0,r*move_step/tv_good)


def left(r=1):
	print 'left',r
	move(0.0,rv_good,r*(math.pi/2)/rv_good)


def right(r=1):
	print 'right',r
	move(0.0,-rv_good,r*(math.pi/2)/rv_good)


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



	


