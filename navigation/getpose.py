import math, time

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

import tf

TOPIC_odom = '/odom'
TOPIC_amcl_pose = '/amcl_pose'
FRAME_map = '/laser_frame'
FRAME_base = '/base_frame'

odom_robot_pose = None
map_robot_pose = None
odomcount = 0 
odomframe = ''


def odom_cb(data):
    global odom_robot_pose, odomcount, odomframe
    if (odom_robot_pose is None):
        odom_robot_pose = [0,0,0]
    odom_robot_pose[0] = data.pose.pose.position.x
    odom_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    odom_robot_pose[2] = euler[2] # yaw
    odomcount += 1
    odomframe = data.header.frame_id

def localizer_cb(data):
    global map_robot_pose
    if (map_robot_pose is None):
        map_robot_pose = [0,0,0]
    map_robot_pose[0] = data.pose.pose.position.x
    map_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    map_robot_pose[2] = euler[2] # yaw



        

def DEG(a):
    return a*180.0/math.pi

def pose_str(p):
    return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))


# main
if __name__ == "__main__":

    rospy.init_node('getpose', disable_signals=True)
    rospy.sleep(1)

    odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, odom_cb)
    localizer_sub = rospy.Subscriber(TOPIC_amcl_pose, PoseWithCovarianceStamped, localizer_cb)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and odom_robot_pose is None \
            and map_robot_pose is None:
        rate.sleep()

    print("Map  Robot pose: %s"  %(pose_str(map_robot_pose)))
    print("Odom Robot pose: %s"  %(pose_str(odom_robot_pose)))

    odom_sub.unregister()
    localizer_sub.unregister()



