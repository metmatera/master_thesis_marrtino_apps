# autostart localizer

from __future__ import print_function

import sys, os, time, math
import rospy
import rosnode
import tf
import argparse

from nav_msgs.msg import Odometry

TOPIC_GROUND_TRUTH = '/base_pose_ground_truth'

def get_ROS_nodes():
    nodes = None
    try:
        nodes = rosnode.get_node_names()
    except Exception as e:
        print(e)
    return nodes

def noderunning(nodename):
    nodes = get_ROS_nodes()
    #print(nodes)
    return nodename in nodes

gt_robot_pose = None

def groundtruth_cb(data):
    global gt_robot_pose
    if gt_robot_pose is None:
        gt_robot_pose = [0,0,0]
    gt_robot_pose[0] = data.pose.pose.position.x
    gt_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    gt_robot_pose[2] = euler[2] # yaw



### main ###

def main(args):
    global gt_robot_pose

    rospy.init_node('autostart_localizer')

    #check if stageros is running
    stagerun = noderunning('/stageros')
    print('stageros running: %r' %(stagerun))

    #check if map_server already running
    msrun = noderunning('/map_server')
    print('map_server running: %r' %(msrun))

    '''
    # get map name
    mapname = 'map' # default map name
    if len(argv)>1:
        mapname = argv[1] # map name
    else:
        try:
            mapname = rospy.get_param('/mapname')
        except:
            pass
    '''

    print('map name: %s' %args.mapname)

    if not msrun:
        #   check if mapname file exists - if not ERROR
        print("TODO: check if map file %s exists" %args.mapname)

    mapdir = args.mapdir
    if mapdir == None:
        mapdir = os.getenv("MARRTINO_APPS_HOME")+"/mapping/maps"
        # mapdir = os.getenv("HOME")+"/playground/maps"


    #get robot current pose from topic /base_pose_ground_truth
    gt_sub = rospy.Subscriber(TOPIC_GROUND_TRUTH, Odometry, groundtruth_cb)

    print("Waiting for ground truth pose...")
    rospy.sleep(1)

    gt_sub.unregister()

    if gt_robot_pose is None and args.initpose != None:
        pp = args.initpose.split()
        gt_robot_pose = [0,0,0]
        gt_robot_pose[0]=float(pp[0])
        gt_robot_pose[1]=float(pp[1])
        gt_robot_pose[2]=float(pp[2])/180.0*math.pi # rad

    print("Current robot pose: %r" %(gt_robot_pose))

    # start mapserver
    if args.mapserveronly:
        rstr = "rosrun map_server map_server %s/%s.yaml" %(mapdir,args.mapname)
    #start localizer
    elif (msrun and stagerun):
        rstr = "roslaunch amcl.launch use_mapserver:=false "
    else:
        rstr = "roslaunch amcl.launch mapsdir:=%s map_name:=%s use_mapserver:=true " %(mapdir,args.mapname)

    if gt_robot_pose is not None:
        rstr += " initial_pose_x:=%.1f initial_pose_y:=%.1f  initial_pose_a:=%.3f" \
            %(gt_robot_pose[0], gt_robot_pose[1], gt_robot_pose[2])

    print(rstr)
    os.system(rstr)


# Use startloc.py <mapname> [-mapdir <DIR>] [-initpose <X Y TH_DEG>]

if __name__=='__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("mapname", type=str, default="map",  nargs='?', help="Map name")                  
    parser.add_argument("-mapdir", type=str, default=None,   help="Map folder")
    parser.add_argument("-initpose", type=str, default=None,
        help="Init pose in the form 'X Y Th_deg' (default: None)")
    parser.add_argument('--mapserveronly', default = False, action ='store_true', 
        help='Start only map_server node')

    args = parser.parse_args()

    main(args)
