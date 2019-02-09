import math
import argparse

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

ACTION_move_base = '/move_base'
TOPIC_odom = '/odom'
TOPIC_cmd_vel = '/cmd_vel'
FRAME_map = 'map'
FRAME_base = 'base_frame'

odom_robot_pose = [0, 0, 0]
map_robot_pose = [0, 0, 0]
robot_vel = [0, 0]
target_pose = [0, 0, 0]
odomcount = 0 
odomframe = ''

listener = None

def odom_cb(data):
    global odom_robot_pose, odomcount, odomframe
    odom_robot_pose[0] = data.pose.pose.position.x
    odom_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    odom_robot_pose[2] = euler[2] # yaw
    odomcount += 1
    odomframe = data.header.frame_id


def get_robot_pose():
    global map_robot_pose
    try:
        (trans,rot) = listener.lookupTransform(FRAME_map, FRAME_base, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
    map_robot_pose[0] = trans[0]
    map_robot_pose[1] = trans[1]
    map_robot_pose[2] = yaw


def dist_from_goal():
    global map_robot_pose, target_pose
    get_robot_pose()
    return math.sqrt(math.pow(map_robot_pose[0]-target_pose[0],2)+math.pow(map_robot_pose[1]-target_pose[1],2))



def cmdvel_cb(data):
    global robot_vel
    robot_vel[0] = data.linear.x
    robot_vel[1] = data.angular.z

ac_movebase = None 
move_base_running = False

def exec_movebase(gx, gy, gth_deg):
    global ac_movebase, move_base_running
    if (ac_movebase == None):
        ac_movebase = actionlib.SimpleActionClient(ACTION_move_base,MoveBaseAction)
    ac_movebase.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = FRAME_map
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = gx
    goal.target_pose.pose.position.y = gy
    yaw = gth_deg/180.0*math.pi
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    ac_movebase.send_goal(goal)
    move_base_running = True
    print('Moving to %.2f %.2f %.2f' %(gx,gy,gth_deg))

    ac_movebase.wait_for_result()  # blocking until goal is reached

    print('Move action completed')
    move_base_running = False


def stop_movebase():
    global ac_movebase, move_base_running
    if (ac_movebase == None):
        ac_movebase = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    ac_movebase.wait_for_server()
    ac_movebase.cancel_all_goals()
    move_base_running = False


thr_movebase = None

def start_movebase(tpose):
    global thr_movebase, target_pose
    target_pose = tpose
    thr_movebase = Thread(target=exec_movebase, args=(target_pose[0], target_pose[1], target_pose[2],))
    thr_movebase.start()


def join_movebase():
    global thr_movebase
    thr_movebase.join()




def do_move(target_pose):

    start_movebase(target_pose)

    delay = 0.25 # sec
    rate = rospy.Rate(1/delay) # Hz

    dorun = True

    # wait for actual start
    while dorun and not move_base_running and not rospy.is_shutdown():
        try:
            rate.sleep()
        except:  # CTRL-C to stop
            dorun = False

    # wait for move_base termination
    while dorun and move_base_running and not rospy.is_shutdown():
        try:
            rate.sleep()
        except:  # CTRL-C to stop
            dorun = False

    stop_movebase()

    join_movebase()

    if dist_from_goal()<0.5:
        print('Goal reached')
    else:
        print('Goal not reached')

# main
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='move_base navigation')
    parser.add_argument('GX', type=float, help='Goal X')
    parser.add_argument('GY', type=float, help='Goal Y')
    parser.add_argument('GTH', type=float, help='Goal Theta [deg]')

    args = parser.parse_args()
    target_pose = [args.GX,args.GY,args.GTH]

    rospy.init_node('move', disable_signals=True)

    listener = tf.TransformListener()
    odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, odom_cb)
    cmd_vel_sub = rospy.Subscriber(TOPIC_cmd_vel, Twist, cmdvel_cb)

    do_move(target_pose)

    odom_sub.unregister()
    cmd_vel_sub.unregister()

    print('Quit')






