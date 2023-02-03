import math, time, sys
from math import sin, cos, atan2

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Pose, Twist

import tf

TOPIC_cmdvel = '/cmd_vel'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

def pose_str(p):
    return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))

# main
if __name__ == "__main__":

	rospy.init_node('setcmdvel', disable_signals=True)
	rate = rospy.Rate(10)
	rate.sleep()

	if len(sys.argv)<4:
		print("%s <name> <v> <omega>" %(sys.argv[0]))
		sys.exit(0)

	obstacle = sys.argv[1]
	v = float(sys.argv[2])
	omega = float(sys.argv[3])

	TOPIC_cmdvel = "/" + obstacle + TOPIC_cmdvel

	cmdvel_pub = rospy.Publisher(TOPIC_cmdvel, Twist, queue_size=10, latch=True)
	t = Twist()
	
	i = 0
	while (i < 10):
	
		t.linear.x = v
		t.linear.y = 0
		t.linear.z = 0
		t.angular.x = 0
		t.angular.y = 0
		t.angular.z = omega

		print("Setting cmd_vel %s: %r" %(TOPIC_cmdvel, t))

		cmdvel_pub.publish(t)
		i += 1
		
		rate.sleep()
		
		
