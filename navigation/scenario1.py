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

TOPIC_cmdvel_1 = '/human1/cmd_vel'
TOPIC_cmdvel_2 = '/human2/cmd_vel'
TOPIC_cmdvel_3 = '/human3/cmd_vel'
TOPIC_cmdvel_4 = '/human4/cmd_vel'
TOPIC_cmdvel_5 = '/human5/cmd_vel'
TOPIC_cmdvel_6 = '/human6/cmd_vel'
TOPIC_cmdvel_7 = '/human7/cmd_vel'
TOPIC_cmdvel_8 = '/human8/cmd_vel'
TOPIC_cmdvel_9 = '/human9/cmd_vel'

TOPIC_gt_1 = '/human1/base_pose_ground_truth'
TOPIC_gt_2 = '/human2/base_pose_ground_truth'
TOPIC_gt_3 = '/human3/base_pose_ground_truth'
TOPIC_gt_4 = '/human4/base_pose_ground_truth'
TOPIC_gt_5 = '/human5/base_pose_ground_truth'
TOPIC_gt_6 = '/human6/base_pose_ground_truth'
TOPIC_gt_7 = '/human7/base_pose_ground_truth'
TOPIC_gt_8 = '/human8/base_pose_ground_truth'
TOPIC_gt_9 = '/human9/base_pose_ground_truth'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

def pose_str(p):
	if p is None:
		return None
    	else:
    		return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))
    
ground_truth_pose = None
gtps = [None, None, None, None, None, None, None, None, None]
def groundtruth_cb(data, idx):
    global ground_truth_pose, gtps
    ground_truth_pose = gtps[idx-1]
    if (ground_truth_pose is None):
        ground_truth_pose = [0,0,0]
    ground_truth_pose[0] = data.pose.pose.position.x
    ground_truth_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    ground_truth_pose[2] = euler[2] # yaw
    gtps[idx-1] = ground_truth_pose

def allZeroedOrientations(poses):
	for p in poses:
		if (p is None or DEG(p[2]) != 0.0):
			return False
	return True

# main
if __name__ == "__main__":

	rospy.init_node('setcmdvel', disable_signals=True)
	rate = rospy.Rate(90)
	rate.sleep()
	
	# Define all Publishers for setpose
	# Initializing start configuration of people
	setpose_pub_1 = rospy.Publisher(TOPIC_setpose_1, Pose, queue_size=10, latch=True)
	setpose_pub_2 = rospy.Publisher(TOPIC_setpose_2, Pose, queue_size=10, latch=True)
	setpose_pub_3 = rospy.Publisher(TOPIC_setpose_3, Pose, queue_size=10, latch=True)
	setpose_pub_4 = rospy.Publisher(TOPIC_setpose_4, Pose, queue_size=10, latch=True)
	setpose_pub_5 = rospy.Publisher(TOPIC_setpose_5, Pose, queue_size=10, latch=True)
	setpose_pub_6 = rospy.Publisher(TOPIC_setpose_6, Pose, queue_size=10, latch=True)
	setpose_pub_7 = rospy.Publisher(TOPIC_setpose_7, Pose, queue_size=10, latch=True)
	setpose_pub_8 = rospy.Publisher(TOPIC_setpose_8, Pose, queue_size=10, latch=True)
	setpose_pub_9 = rospy.Publisher(TOPIC_setpose_9, Pose, queue_size=10, latch=True)

	# Define all Publishers for cmd_vel
	cmdvel_pub_1 = rospy.Publisher(TOPIC_cmdvel_1, Twist, queue_size=10, latch=True)
	cmdvel_pub_2 = rospy.Publisher(TOPIC_cmdvel_2, Twist, queue_size=10, latch=True)
	cmdvel_pub_3 = rospy.Publisher(TOPIC_cmdvel_3, Twist, queue_size=10, latch=True)
	cmdvel_pub_4 = rospy.Publisher(TOPIC_cmdvel_4, Twist, queue_size=10, latch=True)
	cmdvel_pub_5 = rospy.Publisher(TOPIC_cmdvel_5, Twist, queue_size=10, latch=True)
	cmdvel_pub_6 = rospy.Publisher(TOPIC_cmdvel_6, Twist, queue_size=10, latch=True)
	cmdvel_pub_7 = rospy.Publisher(TOPIC_cmdvel_7, Twist, queue_size=10, latch=True)
	cmdvel_pub_8 = rospy.Publisher(TOPIC_cmdvel_8, Twist, queue_size=10, latch=True)
	cmdvel_pub_9 = rospy.Publisher(TOPIC_cmdvel_9, Twist, queue_size=10, latch=True)
	
	# Define all Subscribers for base_pose_ground_truth
	gt_sub_1 = rospy.Subscriber(TOPIC_gt_1, Odometry, groundtruth_cb, (1))
	gt_sub_2 = rospy.Subscriber(TOPIC_gt_2, Odometry, groundtruth_cb, (2))
	gt_sub_3 = rospy.Subscriber(TOPIC_gt_3, Odometry, groundtruth_cb, (3))
	gt_sub_4 = rospy.Subscriber(TOPIC_gt_4, Odometry, groundtruth_cb, (4))
	gt_sub_5 = rospy.Subscriber(TOPIC_gt_5, Odometry, groundtruth_cb, (5))
	gt_sub_6 = rospy.Subscriber(TOPIC_gt_6, Odometry, groundtruth_cb, (6))
	gt_sub_7 = rospy.Subscriber(TOPIC_gt_7, Odometry, groundtruth_cb, (7))
	gt_sub_8 = rospy.Subscriber(TOPIC_gt_8, Odometry, groundtruth_cb, (8))
	gt_sub_9 = rospy.Subscriber(TOPIC_gt_9, Odometry, groundtruth_cb, (9))
	
	# Init all constructors
	# name (job): [x y th]
	
	# human1 (doctor): [5 13 0]
	p = Pose()
	p.position.x = 5.0
	p.position.y = 13.0
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(0.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_1.publish(p)
	t1 = Twist()
	t1.linear.x = 0
	t1.linear.y = 0
	t1.linear.z = 0
	t1.angular.x = 0
	t1.angular.y = 0
	t1.angular.z = 0
	rate.sleep()
	
	# human2 (nurse): [5.5 14.5 -45]
	p = Pose()
	p.position.x = 5.5
	p.position.y = 14.5
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(-45.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_2.publish(p)
	t2 = Twist()
	t2.linear.x = 0
	t2.linear.y = 0
	t2.linear.z = 0
	t2.angular.x = 0
	t2.angular.y = 0
	t2.angular.z = 0
	rate.sleep()
	
	# human3 (doctor): [7 15 -90]
	p = Pose()
	p.position.x = 7.0
	p.position.y = 15.0
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(-90.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_3.publish(p)
	t3 = Twist()
	t3.linear.x = 0
	t3.linear.y = 0
	t3.linear.z = 0
	t3.angular.x = 0
	t3.angular.y = 0
	t3.angular.z = 0
	rate.sleep()
	
	# human4 (nurse): [8.5 14.5 -135]
	p = Pose()
	p.position.x = 8.5
	p.position.y = 14.5
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(-135.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_4.publish(p)
	t4 = Twist()
	t4.linear.x = 0
	t4.linear.y = 0
	t4.linear.z = 0
	t4.angular.x = 0
	t4.angular.y = 0
	t4.angular.z = 0
	rate.sleep()
	
	# human5 (doctor): [9 13 180]
	p = Pose()
	p.position.x = 9.0
	p.position.y = 13.0
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(180.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_5.publish(p)
	t5 = Twist()
	t5.linear.x = 0
	t5.linear.y = 0
	t5.linear.z = 0
	t5.angular.x = 0
	t5.angular.y = 0
	t5.angular.z = 0
	rate.sleep()
	
	# human6 (nurse): [8.5 11.5 135]
	p = Pose()
	p.position.x = 8.5
	p.position.y = 11.5
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(135.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_6.publish(p)
	t6 = Twist()
	t6.linear.x = 0
	t6.linear.y = 0
	t6.linear.z = 0
	t6.angular.x = 0
	t6.angular.y = 0
	t6.angular.z = 0
	rate.sleep()
	
	# human7 (doctor): [7 11 90]
	p = Pose()
	p.position.x = 7.0
	p.position.y = 11.0
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(90.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_7.publish(p)
	t7 = Twist()
	t7.linear.x = 0
	t7.linear.y = 0
	t7.linear.z = 0
	t7.angular.x = 0
	t7.angular.y = 0
	t7.angular.z = 0
	rate.sleep()
	
	# human8 (nurse): [5.5 11.5 45]
	p = Pose()
	p.position.x = 5.5
	p.position.y = 11.5
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(45.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_8.publish(p)
	t8 = Twist()
	t8.linear.x = 0
	t8.linear.y = 0
	t8.linear.z = 0
	t8.angular.x = 0
	t8.angular.y = 0
	t8.angular.z = 0
	rate.sleep()
	
	# human9 (nurse by bed6): [16.83 22.09 -135]
	p = Pose()
	p.position.x = 16.83
	p.position.y = 22.09
	p.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, RAD(-135.0))
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	setpose_pub_9.publish(p)
	t9 = Twist()
	t9.linear.x = 0
	t9.linear.y = 0
	t9.linear.z = 0
	t9.angular.x = 0
	t9.angular.y = 0
	t9.angular.z = 0
	rate.sleep()
	
	rospy.sleep(3)
	
	# Implementation
	T = [t1,t2,t3,t4,t5,t6,t7,t8,t9]
	P = [cmdvel_pub_1, cmdvel_pub_2, cmdvel_pub_3, cmdvel_pub_4, cmdvel_pub_5, cmdvel_pub_6, cmdvel_pub_7, cmdvel_pub_8, cmdvel_pub_9]
	
	while not rospy.is_shutdown() and not allZeroedOrientations(gtps):
		
		for i in range(len(T)):
			T[i].angular.z = math.pi
			P[i].publish(T[i])
			print("Ground truth pose of human%d: %s"  %(i+1,pose_str(gtps[i])))
		print("-"*55)
		
		rate.sleep()
			
	
	
	
	
	
	
	
	
	
	
		
