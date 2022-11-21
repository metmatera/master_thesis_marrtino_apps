import math, time, sys
from math import sin, cos, atan2

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Pose, Twist

import tf

TOPIC_setpose_1 = '/human1/setpose'
TOPIC_setpose_2 = '/human2/setpose'
TOPIC_setpose_3 = '/human3/setpose'
TOPIC_setpose_4 = '/human4/setpose'
TOPIC_setpose_5 = '/human5/setpose'
TOPIC_setpose_6 = '/human6/setpose'
TOPIC_setpose_7 = '/human7/setpose'
TOPIC_setpose_8 = '/human8/setpose'
TOPIC_setpose_9 = '/human9/setpose'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

def quaternion(deg):
	return tf.transformations.quaternion_from_euler(0, 0, RAD(deg))

def setOrientation(pose, theta):
	q = quaternion(theta)
	pose.orientation.x = q[0]
	pose.orientation.y = q[1]
	pose.orientation.z = q[2]
	pose.orientation.w = q[3]
	
				
def alignedOrientations(thetas, thetas_d):
	for th, th_d in zip(thetas, thetas_d):
		if (th != th_d and th % 360 != th_d):
			return False
	return True

	
def pose_str(p):
	return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))

# main
if __name__ == "__main__":

	rospy.init_node('heart_attack_test', disable_signals=True)
	rate = rospy.Rate(100)
	rate.sleep()
	
	# Define all Publishers for /human{i}/setpose
	setpose_pub_1 = rospy.Publisher(TOPIC_setpose_1, Pose, queue_size=10, latch=True)
	setpose_pub_2 = rospy.Publisher(TOPIC_setpose_2, Pose, queue_size=10, latch=True)
	setpose_pub_3 = rospy.Publisher(TOPIC_setpose_3, Pose, queue_size=10, latch=True)
	setpose_pub_4 = rospy.Publisher(TOPIC_setpose_4, Pose, queue_size=10, latch=True)
	setpose_pub_5 = rospy.Publisher(TOPIC_setpose_5, Pose, queue_size=10, latch=True)
	setpose_pub_6 = rospy.Publisher(TOPIC_setpose_6, Pose, queue_size=10, latch=True)
	setpose_pub_7 = rospy.Publisher(TOPIC_setpose_7, Pose, queue_size=10, latch=True)
	setpose_pub_8 = rospy.Publisher(TOPIC_setpose_8, Pose, queue_size=10, latch=True)
	setpose_pub_9 = rospy.Publisher(TOPIC_setpose_9, Pose, queue_size=10, latch=True)
	
	# Init all constructors
	# name (job): [x y th]
	
	print("Setting initial configurations...")
	
	# human1 (doctor): [5 13 0]
	p1 = Pose()
	p1.position.x = 14.8
	p1.position.y = 10.11
	p1.position.z = 0
	th1 = 30.0
	setOrientation(p1, th1)
	setpose_pub_1.publish(p1)
	rate.sleep()
	
	# human2 (nurse): [5.5 14.5 -45]
	p2 = Pose()
	p2.position.x = 6.78
	p2.position.y = 12.80
	p2.position.z = 0
	th2 = 90.0
	setOrientation(p2, th2)
	setpose_pub_2.publish(p2)
	rate.sleep()
	
	# human3 (doctor): [7 15 -90]
	p3 = Pose()
	p3.position.x = 18.77
	p3.position.y = 15.29
	p3.position.z = 0
	th3 = 60.0
	setOrientation(p3, th3)
	setpose_pub_3.publish(p3)
	rate.sleep()
	
	# human4 (nurse): [8.5 14.5 -135]
	p4 = Pose()
	p4.position.x = 9.82
	p4.position.y = 8.97
	p4.position.z = 0
	th4 = 0.0
	setOrientation(p4, th4)
	setpose_pub_4.publish(p4)
	rate.sleep()
	
	# human5 (doctor): [9 13 180]
	p5 = Pose()
	p5.position.x = 14.43
	p5.position.y = 7.71
	p5.position.z = 0
	th5 = 180.0
	setOrientation(p5, th5)
	setpose_pub_5.publish(p5)
	rate.sleep()
	
	# human6 (nurse): [8.5 11.5 135]
	p6 = Pose()
	p6.position.x = 14.21
	p6.position.y = 5.44
	p6.position.z = 0
	th6 = 135.0
	setOrientation(p6, th6)
	setpose_pub_6.publish(p6)
	rate.sleep()
	
	# human7 (doctor): [7 11 90]
	p7 = Pose()
	p7.position.x = 6.87
	p7.position.y = 15.43
	p7.position.z = 0
	th7 = 30.0
	setOrientation(p7, th7)
	setpose_pub_7.publish(p7)
	rate.sleep()
	
	# human8 (nurse): [5.5 11.5 45]
	p8 = Pose()
	p8.position.x = 5.68
	p8.position.y = 8.31
	p8.position.z = 0
	th8 = 45.0
	setOrientation(p8, th8)
	setpose_pub_8.publish(p8)
	rate.sleep()
	
	# human9 (nurse by bed6): [16.83 22.09 -135]
	p9 = Pose()
	p9.position.x = 16.74
	p9.position.y = 20.53
	p9.position.z = 0
	th9 = -135.0
	setOrientation(p9, th9)
	setpose_pub_9.publish(p9)
	rate.sleep()
	
	print("Done")
	rospy.sleep(2)
	
	# IMPLEMENTATION
	P = [p1,p2,p3,p4,p5,p6,p7,p8,p9]
	Th = [th1,th2,th3,th4,th5,th6,th7,th8,th9]
	Pubs = [setpose_pub_1, setpose_pub_2, setpose_pub_3, setpose_pub_4, setpose_pub_5, setpose_pub_6, setpose_pub_7, setpose_pub_8, setpose_pub_9]
	
	print("All people rotating until alignment...")
	thetas_d = []
	for i in range(len(P)-1):
		theta_d = round(DEG(math.atan2(p9.position.y - P[i].position.y, p9.position.x - P[i].position.x)))
		a, _ = divmod(theta_d, 5)
		thetas_d.append(a * 5)
	print("Desired orientations: ", thetas_d)
	
	signs = [-1 if abs(th) > abs(th_d) else 1 for th, th_d in zip(Th[:8], thetas_d)]

	while not alignedOrientations(Th[:8], thetas_d):
		for i in range(len(P)-1): # human9 excluded
			if (Th[i] != thetas_d[i] and Th[i] % 360.0 != thetas_d[i] % 360.0):
				Th[i] += 5.0 * signs[i]
				setOrientation(P[i], Th[i])
				Pubs[i].publish(P[i])
				#print("%d: %.2f" %(i+1, Th[i]))
		#print("-"*55)
		rate.sleep()
	print("Done")
	
	print("All people moving...")
	duration, step = 150, 70
	for i in range(duration):
		for i in range(len(P)-1): # human9 excluded
			P[i].position.x += (p9.position.x - P[i].position.x) / step
			P[i].position.y += (p9.position.y - P[i].position.y) / step
			Pubs[i].publish(P[i])
		rate.sleep()
		#rospy.sleep(0.1)
	print("Done")
	
	
	
	
	
	
	
	
			
