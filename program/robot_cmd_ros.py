#!/usr/bin/env python

import time
import os
import socket
import math

import rospy
import tf

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from apriltags_ros.msg import AprilTagDetectionArray

AUDIO_SERVER_IP = '127.0.0.1'
AUDIO_SERVER_PORT = 9001
assock = None

use_robot = True
use_obstacle_avoidance = False

# Good values
tv_good = 0.2
rv_good = 0.8

move_step = 0.5;


def setMoveStep(x):
	global move_step
	move_step=x


def setMaxSpeed(x,r):
	global tv_good
	global rv_good
	tv_good=x
	rv_good=r

def enableObstacleAvoidance():
	global use_obstacle_avoidance
	use_obstacle_avoidance = True

# Condition Variables and Functions

tag_trigger_ = False
tag_id_ = -1
tag_distance_ = 0
tag_count = 25

def tag_trigger():
	global tag_trigger_
	return tag_trigger_

def tag_id():
	global tag_id_
	return tag_id_

def tag_distance():
	global tag_distance_
	return tag_distance_

laser_center_dist_ = 10

def laser_center_distance():
	global laser_center_dist_
	return laser_center_dist_

robot_pose_ = [0,0,0]

def get_robot_pose():
	return robot_pose_



# ROS publishers/subscribers
cmd_pub = None # cmd_vel publisher
tag_sub = None # tag_detection subscriber
laser_sub = None # laser subscriber
odom_sub = None  # odom subscriber



# ROS Callback functions


def tag_cb(data):
	global tag_trigger_, tag_count, tag_id_, tag_distance_
	v = data.detections
	if (len(v)>0):
		tag_id_ = v[0].id
		tag_distance_ = v[0].pose.pose.position.z
		tag_trigger_ = True
		tag_count = 3 # about seconds
		# print 'tag ',tag_id_,' distance ',tag_distance_
		# print 'tag trigger = ',tag_trigger_
	else:
		if (tag_trigger):
			tag_count = tag_count - 1
			# print 'tag count = ',tag_count
			if (tag_count==0):
				tag_trigger_ = False


def laser_cb(data):
	global laser_center_dist_
	n = len(data.ranges)
	laser_center_dist_ = data.ranges[n/2]


def odom_cb(data):
	global robot_pose_
	robot_pose_[0] = data.pose.pose.position.x
	robot_pose_[1] = data.pose.pose.position.y
	o = data.pose.pose.orientation
	q = (o.x, o.y, o.z, o.w)
	euler = tf.transformations.euler_from_quaternion(q)
	robot_pose_[2] = euler[2] # yaw






# Begin/end

def begin():
	global assock
	global cmd_pub, tag_sub, laser_sub
	print 'begin'

	if (use_robot):
		print "Robot enabled"
		rospy.init_node('robot_cmd')
		cmd_vel_topic = 'cmd_vel'
		if (use_obstacle_avoidance):
			cmd_vel_topic = 'desired_cmd_vel'

		cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
		tag_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_cb)
		laser_sub = rospy.Subscriber('scan', LaserScan, laser_cb)
		odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)

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

def setSpeed(lx,az,tm):
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


def stop():
	print 'stop'
	setSpeed(0,0,0.5)


def forward(r=1):
	global tv_good
	print 'forward',r
	setSpeed(tv_good,0.0,r*move_step/tv_good)
	

def backward(r=1):
	print 'backward',r
	setSpeed(-tv_good,0.0,r*move_step/tv_good)


def left(r=1):
	print 'left',r
	setSpeed(0.0,rv_good,r*(math.pi/2)/rv_good)


def right(r=1):
	print 'right',r
	setSpeed(0.0,-rv_good,r*(math.pi/2)/rv_good)


# Wait

def wait(r=1):
	print 'wait',r
	if (r==0):
		time.sleep(0.1)
	else:
		for i in range(0,r):
			time.sleep(1)


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



	


