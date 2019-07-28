#!/usr/bin/env python

import time
import os
import socket
import math
import sys
import rospy
import tf
import actionlib
from threading import Thread

import cv2

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rococo_navigation.msg import FollowPersonAction, FollowPersonGoal
from cv_bridge import CvBridge, CvBridgeError

try:
    from apriltags_ros.msg import AprilTagDetectionArray
    AprilTagFound = True
except:
    print("apriltag_ros not found")
    AprilTagFound = False

AUDIO_SERVER_IP = '127.0.0.1'
AUDIO_SERVER_PORT = 9001
assock = None

use_robot = True
use_audio = True

robot_initialized = False
stop_request = False

# Topic names

TOPIC_tag_detections = 'tag_detections'
TOPIC_scan = 'scan'
TOPIC_amcl_pose = 'amcl_pose'
TOPIC_cmd_vel = 'cmd_vel'
TOPIC_desired_cmd_vel = 'desired_cmd_vel'
TOPIC_odom = 'odom'
ACTION_move_base = 'move_base'
TOPIC_sonar_0 = '/sonar_0' 
TOPIC_sonar_1 = '/sonar_1'
TOPIC_sonar_2 = '/sonar_2'
TOPIC_sonar_3 = '/sonar_3'



# Good values
tv_good = 0.2
rv_good = 0.8
tv_min = 0.1
rv_min = 0.2

move_step = 1.0

# robot pose from odometry
odom_robot_pose = None
# robot pose from localization
loc_robot_pose = None

move_base_running = False
ac_movebase = None 

def setMoveStep(x):
    global move_step
    move_step=x


def setMaxSpeed(x,r):
    global tv_good
    global rv_good
    tv_good=x
    rv_good=r


def setRobotNamePrefix(prefix):
    global TOPIC_tag_detections,TOPIC_scan,TOPIC_amcl_pose,TOPIC_cmd_vel,TOPIC_desired_cmd_vel,TOPIC_odom,ACTION_move_base

    TOPIC_tag_detections = prefix+'/tag_detections'
    TOPIC_scan = prefix+'/scan'
    TOPIC_amcl_pose = prefix+'/amcl_pose'
    TOPIC_cmd_vel = prefix+'/cmd_vel'
    TOPIC_desired_cmd_vel = prefix+'/desired_cmd_vel'
    TOPIC_odom = prefix+'/odom'
    ACTION_move_base = prefix+'/move_base'
    TOPIC_sonar_0 = prefix+'/sonar_0' 
    TOPIC_sonar_1 = prefix+'/sonar_1'
    TOPIC_sonar_2 = prefix+'/sonar_2'
    TOPIC_sonar_3 = prefix+'/sonar_3'


def setAudioConnection(ip, port=9001):
    global AUDIO_SERVER_IP, AUDIO_SERVER_PORT
    AUDIO_SERVER_IP = ip
    AUDIO_SERVER_PORT = port

PARAM_gbnEnabled = '/gradientBasedNavigation/gbnEnabled'

def enableObstacleAvoidance(value=True):
    rospy.set_param(PARAM_gbnEnabled, value)


def robot_stop_request(): # stop until next begin()
    global stop_request
    stop_request = True
    if (use_robot):
        stop()
    print("stop request")


# Condition Variables and Functions

tag_trigger_ = False
tag_id_ = -1
tag_distance_ = 0
tag_angle_ = 0
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

def tag_angle():
    global tag_angle_
    return tag_angle_

laser_center_dist = 10
laser_left_dist = 10
laser_right_dist = 10
laser_back_dist = 10

def laser_center_distance():
    global laser_center_dist
    return laser_center_dist

def getRobotPose():
    return get_robot_pose()

def get_robot_pose(): # returns [x,y,theta]
    global odom_robot_pose, loc_robot_pose
    if (loc_robot_pose != None):
        return list(loc_robot_pose)
    else:
        return list(odom_robot_pose)

def obstacleDistance(direction=0):
    return obstacle_distance(direction=0)

def obstacle_distance(direction=0):
    global laser_center_dist, laser_left_dist, laser_right_dist, laser_back_dist
    if (direction==0): #front
        return laser_center_dist
    elif (direction==90): #left
        return laser_left_dist
    elif (direction==-90 or direction==270): # right
        return laser_right_dist
    elif (abs(direction)==180): # back
        return laser_back_dist


def distance(p1,p2):
    dx = p1[0]-p2[0]
    dy = p1[1]-p2[1]
    dx2 = dx*dx
    dy2 = dy*dy
    return math.sqrt(dx2+dy2)


# ROS param access

def set_global_param(var, value):
    param = '/MARRtino/params/'+var
    now = rospy.Time.now()
    pd = {} 
    pd['value'] = value
    pd['timestamp'] = now.secs
    rospy.set_param(param, pd)


def get_global_param(var):
    param = '/MARRtino/params/'+var
    value = ''
    if rospy.has_param(param):
        value = rospy.get_param(param,'')
    return value

def del_global_param(var):
    param = '/MARRtino/params/'+var
    if rospy.has_param(param):
        rospy.delete_param(param)


def event():
    pd = get_global_param('event')
    if (pd==''):
        return ''
    now = rospy.Time.now()
    if (now.secs - pd['timestamp'] < 5): # last 5 seconds
        del_global_param('event')
        return pd['value']
    else:
        return ''

# ROS publishers/subscribers
cmd_pub = None # cmd_vel publisher
tag_sub = None # tag_detection subscriber
laser_sub = None # laser subscriber
odom_sub = None  # odom subscriber
localizer_sub = None
sonar_sub_0 = None
sonar_sub_1 = None
sonar_sub_2 = None
sonar_sub_3 = None


# ROS Callback functions


def tag_cb(data):
    global tag_trigger_, tag_count, tag_id_, tag_distance_, tag_angle_
    v = data.detections
    if (len(v)>0):
        tag_id_ = v[0].id
        tag_distance_ = v[0].pose.pose.position.z
        tag_angle_ = math.atan2(-v[0].pose.pose.position.x,v[0].pose.pose.position.z)*180.0/math.pi

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
                tag_id_ = -1
                tag_distance_ = 0
                tag_angle_ = 0



def laser_cb(data):
    global laser_center_dist, laser_left_dist, laser_right_dist, laser_back_dist
    nc = len(data.ranges)/2
    nr = int((data.angle_max - math.pi/2)/data.angle_increment)
    nl = len(data.ranges) - nr
    laser_center_dist = min(data.ranges[nc-10:nc+10])
    try:
        laser_left_dist = min(data.ranges[nl-10:nl+10])
        laser_right_dist = min(data.ranges[nr-10:nr+10])
    except:
        pass
        #laser_left_dist = -1
        #laser_right_dist = -1
    #print("angle min %.3f max %.3f inc %.6f" %(data.angle_min, data.angle_max, data.angle_increment))
    #print("center %.3f left %.3f right %.3f" %(laser_center_dist, laser_left_dist, laser_right_dist))


def sonar_cb(data):
    global laser_center_dist, laser_left_dist, laser_right_dist, laser_back_dist
    if(data.range < data.max_range):
    	r = (data.range*0.75)/0.265 #scale the value of the range in meters
        if(data.header.frame_id == "/sonar_frame_0"): # front
            laser_center_dist = r
        elif(data.header.frame_id == "/sonar_frame_1"): # right
            laser_right_dist = r
        elif(data.header.frame_id == "/sonar_frame_3"): # left
            laser_left_dist = r
        elif(data.header.frame_id == "/sonar_frame_2"): # back
            laser_back_dist = r


def odom_cb(data):
    global odom_robot_pose
    if (odom_robot_pose is None):
        odom_robot_pose = [0,0,0]
    odom_robot_pose[0] = data.pose.pose.position.x
    odom_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    odom_robot_pose[2] = euler[2] # yaw


def localizer_cb(data):
    global loc_robot_pose
    if (loc_robot_pose is None):
        loc_robot_pose = [0,0,0]
    loc_robot_pose[0] = data.pose.pose.position.x
    loc_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    loc_robot_pose[2] = euler[2] # yaw


cvbridge = None
cvimage = None

def image_cb(data):
    global cvbridge, cvimage
    # Convert image to OpenCV format
    try:
        if cvbridge is None:
            cvbridge = CvBridge()
        cvimage = cvbridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)



# select topic of type sensor_msgs/Image
def autoImageTopic():
    topics = rospy.get_published_topics()
    for t in topics:
        if t[1]=='sensor_msgs/Image' and 'depth' not in t[0] and '/ir/' not in t[0]:
            return t[0]
    return None


# Audio client

run_audio_connect = True
audio_connected = False

def audio_connect_thread():
    global run_audio_connect, assock
    print("Audio enabled, Connecting...")
    run_audio_connect = True
    timeout = 5
    while run_audio_connect and timeout>0:
        assock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            assock.connect((AUDIO_SERVER_IP, AUDIO_SERVER_PORT))
            print("Audio connected.")
            run_audio_connect = False
            audio_connected = True
        except:
            print("Cannot connect to audio server %s:%d" %(AUDIO_SERVER_IP, AUDIO_SERVER_PORT))
            time.sleep(1)
            timeout -= 1
    run_audio_connect = False

# Begin/end

def begin(nodename='robot_cmd', use_desired_cmd_vel=False):
    global cmd_pub, tag_sub, laser_sub, sonar_sub_0, sonar_sub_1, sonar_sub_2, sonar_sub_3
    global odom_robot_pose, robot_initialized, stop_request
    global use_robot, use_audio, audio_connected

    print 'begin'

    stop_request = False

    if (use_audio and not audio_connected):
        # Run audio connection thread
        t = Thread(target=audio_connect_thread, args=())
        t.start()
        time.sleep(0.5)

    if (robot_initialized):
        return

    # blocking function if roscore not available !!!
    # do not throw exception
    rospy.init_node(nodename,  disable_signals=True)

    if AprilTagFound:
        tag_sub = rospy.Subscriber(TOPIC_tag_detections, AprilTagDetectionArray, tag_cb)
    laser_sub = rospy.Subscriber(TOPIC_scan, LaserScan, laser_cb)
    sonar_sub_0 = rospy.Subscriber(TOPIC_sonar_0, Range, sonar_cb)
    sonar_sub_1 = rospy.Subscriber(TOPIC_sonar_1, Range, sonar_cb)
    sonar_sub_2 = rospy.Subscriber(TOPIC_sonar_2, Range, sonar_cb)
    sonar_sub_3 = rospy.Subscriber(TOPIC_sonar_3, Range, sonar_cb)
    localizer_sub = rospy.Subscriber(TOPIC_amcl_pose, PoseWithCovarianceStamped, localizer_cb)

    if (use_robot):
        print("Robot enabled")
        cmd_vel_topic = TOPIC_cmd_vel
        if (use_desired_cmd_vel):
            cmd_vel_topic = TOPIC_desired_cmd_vel
        cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, odom_cb)

        print("Waiting for robot pose... (5 seconds)")
        delay = 0.25 # sec
        rate = rospy.Rate(1/delay) # Hz
        try:
            rate.sleep()
            timeout = 5 #seconds
            while (odom_robot_pose is None and timeout>0):
                rate.sleep()
                timeout -= delay
        except KeyboardInterrupt:
            pass
        if (odom_robot_pose is None):
            print("Robot pose not received. Using [0,0,0]")
            odom_robot_pose = [0,0,0]  # default value
        robot_initialized = True



def end():
    global robot_initialized, stop_request

    if not robot_initialized:
        return

    print 'end'    

    if (use_robot):
        stop()
    stop_request = True

    if (use_audio):
        global run_audio_connect, audio_connected
        run_audio_connect = False
        global assock
        if assock != None:
            assock.close()
            assock=None
            audio_connected = False
    time.sleep(0.5) # make sure stuff ends


# to unregister all the subscribers
def unregisterAll():
    #sub_XXX.unregister()
    pass


sub_image = None

def startCameraGrabber():
    global sub_image
    img_topic = autoImageTopic()
    if img_topic != None:
        print("Image topic: %s" %img_topic)
        sub_image = rospy.Subscriber(img_topic, Image, image_cb)
        time.sleep(1)



def stopCameraGrabber():
    global sub_image
    if sub_image !=  None:
        sub_image.unregister()


def getImage(tmsleep=3):
    global cvimage
    startCameraGrabber() # wait 1 sec for an image
    time.sleep(tmsleep)
    stopCameraGrabber()
    return cvimage


def getWebImage(objcat=None):
    rchomelearnros_import()
    return webimages.take_image(objcat)

# Haar detector
def findCascadeModel():
    trylist = ['/usr/share/opencv/', '/opt/ros/kinetic/share/OpenCV-3.3.1-dev/' ]
    for t in trylist:
        f = t + 'haarcascades/haarcascade_frontalface_default.xml'
        if os.path.isfile(f):
            return cv2.CascadeClassifier(f)
    return None

faceCascade = None

def faceDetection(img):
    global faceCascade
    if faceCascade is None:
        faceCascade = findCascadeModel()
        if faceCascade is None:
            print("ERROR Cannot find Haar cascade model")
            return -1
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect faces in the image
    faces = faceCascade.detectMultiScale(gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30)
    )
    return len(faces)


# rc-home-learn-ros import
rchomelearnros_imported = False
mobilenet_objrec = None 
webimages = None

def rchomelearnros_import():
    global rchomelearnros_imported, mobilenet_objrec, webimages
    if rchomelearnros_imported:
        return

    path = None
    try:
        import rospkg
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        path = rospack.get_path('rc-home-edu-learn-ros')
    except Exception as e:
        #print(e)
        path = os.getenv('HOME')+'/src/rc-home-edu-learn-ros'
    print('rc-home-edu-learn-ros path: %s' %path) 

    try:
        sys.path.append(path+'/rchomeedu_vision/scripts')
        import mobilenet_objrec, webimages
        rchomelearnros_imported = True
    except Exception as e:
        print(e)
        print('Cannot import mobilenet_objrec, webimages modules')


# Object recognition with mobilenet

monet = None

def mobilenetObjrec(img):
    return mobilenet_objrec(img)

def mobilenet_objrec(img):
    global monet
    if monet is None:
        rchomelearnros_import()
        try:
            monet = mobilenet_objrec.MNetObjRec()
        except Exception as e:
            print(e)
            return 'ERROR Mobilenet not available'
    
    r = monet.evalCVImage(img)
    return r


def ready():
    global robot_initialized
    return robot_initialized


# check if program can run now
def marrtinoOK():
    return marrtino_ok()

def marrtino_ok():
    global robot_initialized, stop_request
    return robot_initialized and not stop_request and not rospy.is_shutdown()


# Robot motion

def setSpeed(lx,az,tm,stopend=False):
    global cmd_pub, stop_request

    if (stop_request):
        raise Exception("setSpeed called in stop_request mode")

    delay = 0.1 # sec
    rate = rospy.Rate(1/delay) # Hz
    cnt = 0.0
    msg = Twist()
    msg.linear.x = lx
    msg.angular.z = az
    msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y =  0
    while not rospy.is_shutdown() and cnt<=tm and not stop_request:
        cmd_pub.publish(msg)
        cnt = cnt + delay
        rate.sleep()
    if (stopend):
        msg.linear.x = 0
        msg.angular.z = 0
        cmd_pub.publish(msg)
        rate.sleep()


def stop():
    global move_base_running
    print 'stop'
    if (move_base_running):
        exec_movebase_stop()
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    cmd_pub.publish(msg)
    delay = 0.1 # sec
    try:
        rospy.Rate(10).sleep() # 0.1 sec
    except:
        pass


def forward(r=1):
    global tv_good
    print 'forward',r
    exec_move_REL(move_step*r)
    #setSpeed(tv_good,0.0,r*move_step/tv_good)
    

def backward(r=1):
    print 'backward',r
    exec_move_REL(-move_step*r)
    #setSpeed(-tv_good,0.0,r*move_step/tv_good)


def left(r=1):
    print 'left',r
    exec_turn_REL(90*r)
    # setSpeed(0.0,rv_good,r*(math.pi/2)/rv_good)


def right(r=1):
    print 'right',r
    exec_turn_REL(-90*r)
    #setSpeed(0.0,-rv_good,r*(math.pi/2)/rv_good)

# map frame goto (requires localization)
def goto(gx, gy, gth_deg):
    exec_movebase(gx, gy, gth_deg)


# odom frame direct control (no path planning)
def gotoTarget(gx, gy):
    goto_target(gx, gy)


# odom frame direct control (no path planning)
def goto_target(gx, gy):
    exec_goto_target(gx, gy)

# person follow


def start_follow_person(max_vel = 0.25): # non-blocking
    exec_follow_person_start(max_vel)

def stop_follow_person():
    exec_follow_person_stop()

# Turn

def turn(deg, ref='REL'):
    print('turn %s %.2f' %(ref,deg))
    if ref=='REL':
        exec_turn_REL(deg)
    else:
        exec_turn_ABS(deg)


# Wait

def wait(r=1):
    global stop_request
    #print 'wait',r
    if (r==0):
        time.sleep(0.1)
    else:
        i = 0
        while i<r and not stop_request:
            time.sleep(1)
            i += 1


# Sounds

def sound(name):
    global assock
    print('sound %s' %name)
    try:
        assock.send('SOUND %s\n\r' %name)
        time.sleep(0.5)
        data = assock.recv(80)
        print data
    except:
        pass

def bip(r=1):
    for i in range(0,r):
        sound('bip')

def bop(r=1):
    for i in range(0,r):
        sound('bop')

def boom(r=1):
    for i in range(0,r):
        sound('boom')

# TTS

def say(text, language='en'):
    global assock
    print('say %s [%s]' %(text,language))
    lstr = 'en-US'
    if (language!='en'):
        lstr = language+'-'+language.upper()
    try:
        assock.send('TTS[%s] %s\n\r' %(lstr,text))
        time.sleep(1)
        data = assock.recv(80)
        print data
    except:
        pass

# ASR

def asr():
    global assock, stop_request
    #print 'ASR received: ',
    try:
        data = ''
        while data=='' and not stop_request:
            assock.send('ASR\n\r')  # ask for ASR results
            time.sleep(0.5)
            data = assock.recv(160)
            data = data.strip()
        #print data
        return data
    except:
        return ''


# MODIM

mws = None  # MODIM websocket connection

try:
    sys.path.append(os.getenv('MODIM_HOME')+"/src/GUI")
    from ws_client import *
    mws = ModimWSClient()
except:
    print("No MODIM found!")

# example: show_image('red.jpg', 'default')


def showImage(value, which='default'):
    show_image(value, which)

def show_image(value, which='default'):
    global mws
    if mws!=None:
        cstr = 'im.executeModality("image_%s", "img/%s")' %(which,value)
        #print(cstr)
        r = mws.csend(cstr)
        print(r)


def showText(value, which='default'):
    show_text(value, which)

def show_text(value, which='default'):
    global mws
    if mws!=None:
        cstr = 'im.executeModality("text_%s", "%s")' %(which,value)
        #print(cstr)
        r = mws.csend(cstr)
        print(r)


# Precise move and turn

# Angle functions

def DEG2RAD(a):
    return a*math.pi/180.0

def RAD2DEG(a):
    return a/math.pi*180.0


def NORM_PI(a):
    if (a>math.pi):
        return a-2*math.pi
    elif (a<-math.pi):
        return a+2*math.pi
    else:
        return a

def norm_target_angle(a):
    if (abs(NORM_PI(a-0))<0.3):
        return 0;
    elif (abs(NORM_PI(a-math.pi/2.0))<0.3):
        return math.pi/2;
    elif (abs(NORM_PI(a-math.pi))<0.3):
        return math.pi;
    elif (abs(NORM_PI(a-3*math.pi/2.0))<0.3):
        return -math.pi/2;
    else:
        return a;



def exec_turn_ABS(th_deg):
    global rv_good, rv_min, loc_robot_pose
    current_th = loc_robot_pose[2]
    
    #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), th_deg))
    #print("TURN -- to-normalize RAD: %.1f" %(DEG2RAD(th_deg)))

    target_th = norm_target_angle(DEG2RAD(th_deg))

    #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), RAD2DEG(target_th)))

    ndth = NORM_PI(target_th-current_th)
    dth = abs(ndth)

    #print("TURN -- dTh %.2f norm_PI: %.2f" %(ndth,dth))

    rv_nom = rv_good 
    if (ndth < 0):
        rv_nom *= -1

    last_dth = dth
    #print("TURN -- last_dth %.2f" %(last_dth))

    while (dth>rv_min/8.0 and last_dth>=dth):
        rv = rv_nom
        if (dth<0.8):
            rv = rv_nom*dth/0.8
        if (abs(rv)<rv_min):
            rv = rv_min*rv/abs(rv)
        tv = 0.0
        setSpeed(tv, rv, 0.1, False)
        current_th = loc_robot_pose[2]
        dth = abs(NORM_PI(target_th-current_th))
        if (dth < last_dth or dth>0.3): # to avoid oscillation close to 0
            last_dth = dth
        #print("TURN -- POS: %.1f %.1f %.1f -- targetTh %.1f DTH %.2f -- VEL: %.2f %.2f" %(odom_robot_pose[0], odom_robot_pose[1], RAD2DEG(current_th), RAD2DEG(target_th), RAD2DEG(dth), tv, rv))
    #print("TURN -- dth %.2f - last_dth %.2f" %(dth,last_dth))
    setSpeed(0.0,0.0,0.1)
    #print 'TURN -- end'


def exec_turn_REL(th_deg):
    global rv_good, rv_min, odom_robot_pose
    current_th = odom_robot_pose[2]
    #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), RAD2DEG(current_th) + th_deg))
    #print("TURN -- to-normalize RAD: %.1f" %(current_th + DEG2RAD(th_deg)))
    target_th = norm_target_angle(current_th + DEG2RAD(th_deg))
    #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), RAD2DEG(target_th)))

    rv_nom = rv_good 
    if (th_deg < 0):
        rv_nom *= -1

    dth = abs(NORM_PI(target_th-current_th))

    #print("TURN -- dTh %.2f norm_PI: %.2f" %(current_th-target_th,dth))

    last_dth = dth
    #print("TURN -- last_dth %.2f" %(last_dth))
    while (dth>rv_min/8.0 and last_dth>=dth):
        rv = rv_nom
        if (dth<0.8):
            rv = rv_nom*dth/0.8
        if (abs(rv)<rv_min):
            rv = rv_min*rv/abs(rv)
        tv = 0.0
        setSpeed(tv, rv, 0.1, False)
        current_th = odom_robot_pose[2]
        dth = abs(NORM_PI(target_th-current_th))
        if (dth < last_dth or dth>0.3): # to avoid oscillation close to 0
            last_dth = dth
        #print("TURN -- POS: %.1f %.1f %.1f -- targetTh %.1f DTH %.2f -- VEL: %.2f %.2f" %(odom_robot_pose[0], odom_robot_pose[1], RAD2DEG(current_th), RAD2DEG(target_th), RAD2DEG(dth), tv, rv))
    #print("TURN -- dth %.2f - last_dth %.2f" %(dth,last_dth))
    setSpeed(0.0,0.0,0.1)
    #print 'TURN -- end'


def exec_move_REL(tx):
    global tv_good, odom_robot_pose
    start_pose = list(odom_robot_pose)
    tv_nom = tv_good 
    if (tx < 0):
        tv_nom *= -1
        tx *= -1
    dx = abs(distance(start_pose,odom_robot_pose) - tx)
    while (dx>0.05):
        tv = tv_nom
        if (dx<0.5):
            tv = tv_nom*dx/0.5
        if (abs(tv)<tv_min):
            tv = tv_min*tv/abs(tv)
        rv = 0.0
        setSpeed(tv, rv, 0.1, False)
        pose = odom_robot_pose
        dx = abs(distance(start_pose, pose) - tx)
        #print("MOVE -- POS: %.1f %.1f %.1f -- targetTX %.1f DX %.1f -- VEL: %.2f %.2f" %(pose[0], pose[1], RAD2DEG(pose[2]), tx, dx, tv, rv))
    setSpeed(0.0,0.0,0.1)



def exec_goto_target(gx,gy):
    global tv_good, rv_good, tv_min, rv_min, odom_robot_pose
    goal_pose = [gx,gy,0]
    dx = distance(goal_pose,odom_robot_pose)
    while (dx>0.05):
        tv = tv_good
        if (dx<0.5):
            tv = tv*dx/0.5
        if (abs(tv)<tv_min):
            tv = tv_min*tv/abs(tv)

        current_th = odom_robot_pose[2]
        th_goal = math.atan2(gy-odom_robot_pose[1],gx-odom_robot_pose[0])

        #print("GOTO -- th_target: %.2f" %(RAD2DEG(th_goal)))

        dth = NORM_PI(th_goal-current_th)
        rv = rv_good * dth/abs(dth)
        if (abs(dth)>0.8):
            tv = tv_min
        if (abs(dth)<0.8):
            rv = rv*abs(dth)/0.8
        if (abs(rv)<rv_min):
            rv = rv_min*rv/abs(rv)

        setSpeed(tv, rv, 0.1, False)
        dx = distance(goal_pose, odom_robot_pose)
        #print("GOTO -- POS: %.1f %.1f %.1f -- target %.1f %.1f -- dx: %.1f dth: %.1f -- VEL: %.2f %.2f" %(odom_robot_pose[0], odom_robot_pose[1], RAD2DEG(odom_robot_pose[2]), gx, gy, dx, dth, tv, rv))
    setSpeed(0.0,0.0,0.1)



def exec_movebase(gx, gy, gth_deg):
    global ac_movebase, move_base_running
    if (ac_movebase == None):
        ac_movebase = actionlib.SimpleActionClient(ACTION_move_base,MoveBaseAction)
    ac_movebase.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = gx
    goal.target_pose.pose.position.y = gy
    yaw = gth_deg/180.0*math.pi
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    ac_movebase.send_goal(goal)
    move_base_running = True
    rospy.sleep(0.2)
    wait = ac_movebase.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        #rospy.signal_shutdown("Action server not available!")
    else:
        print ac_movebase.get_result()
    print('Move action completed.')
    move_base_running = False

def exec_movebase_stop():
    global ac_movebase, move_base_running
    if (ac_movebase == None):
        ac_movebase = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    ac_movebase.wait_for_server()
    ac_movebase.cancel_all_goals()
    move_base_running = False


ac_follow_person = None  # action client
follow_person_running = False  # running flag
PERSON_FOLLOW_ACTION = 'follow_person'

def exec_follow_person_start(max_vel):
    global ac_follow_person, follow_person_running
    if (ac_follow_person == None):
        ac_follow_person = actionlib.SimpleActionClient(PERSON_FOLLOW_ACTION,FollowPersonAction)

    print('Waiting for action server %s ...' %PERSON_FOLLOW_ACTION)
    ac_follow_person.wait_for_server()
    print('Done')

    goal = FollowPersonGoal()
    goal.person_id = 0;      # unused so far
    goal.max_vel = max_vel;  # m/s
    ac_follow_person.send_goal(goal)

    print("Follow person START")
    follow_person_running = True


def exec_follow_person_stop():
    global ac_follow_person, follow_person_running
    if (ac_follow_person == None):
        ac_follow_person = actionlib.SimpleActionClient(PERSON_FOLLOW_ACTION,FollowPersonAction)
    print('Waiting for action server %s ...' %PERSON_FOLLOW_ACTION)
    ac_follow_person.wait_for_server()
    print('Done')
    ac_follow_person.cancel_all_goals()

    print("Follow person STOP")

    follow_person_running = False



