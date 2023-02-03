import math, time, sys

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

import tf

TOPIC_ground_truth = '/base_pose_ground_truth'

ground_truth_pose = None

def groundtruth_cb(data):
	global ground_truth_pose
	if (ground_truth_pose is None):
		ground_truth_pose = [0,0,0]
	ground_truth_pose[0] = data.pose.pose.position.x
	ground_truth_pose[1] = data.pose.pose.position.y
	o = data.pose.pose.orientation
	q = (o.x, o.y, o.z, o.w)
	euler = tf.transformations.euler_from_quaternion(q)
	ground_truth_pose[2] = euler[2] # yaw

def DEG(a):
    return a*180.0/math.pi

def pose_str(p):
    return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))


# main
if __name__ == "__main__":

	rospy.init_node('getconfig', disable_signals=True)
	rospy.sleep(1)
	
	if len(sys.argv) == 1:
		print("Get configuration of Marrtino")
		

	elif len(sys.argv) == 2:
		print("Get configuration of %s" %sys.argv[1])
		TOPIC_ground_truth = "/" + sys.argv[1] + TOPIC_ground_truth
		
	else:
		print("Error in arguments")
		sys.exit(0)

	gt_sub = rospy.Subscriber(TOPIC_ground_truth, Odometry, groundtruth_cb)

	rate = rospy.Rate(10)
	timesteps = 10
	while not rospy.is_shutdown() and timesteps>0:
		rate.sleep()
		timesteps -= 1

	if ground_truth_pose is not None:
		print("Ground truth pose: %s"  %(pose_str(ground_truth_pose)))

	gt_sub.unregister()

