import math, time, sys, tf

import numpy as np
from numpy.linalg import norm

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion

TOPIC_human_cmd_vel = '/human1/cmd_vel'
TOPIC_human_setpose = '/human1/setpose'
TOPIC_robot_cmd_vel = '/cmd_vel'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

class Scenario1(object):

    def __init__(self, x0_h, y0_h, v_h, d):
        self.x0_h = x0_h
        self.y0_h = y0_h
        self.v_h = v_h
        self.d = d
        self.cmdvel_pub = rospy.Publisher(TOPIC_human_cmd_vel, Twist, queue_size=10, latch=True)
        self.setpose_pub = rospy.Publisher(TOPIC_human_setpose, Pose, queue_size=1, latch=True)
        self.started = False

    def subscribe(self):
        rospy.init_node('Scenario1')
        rospy.sleep(1)

        robot_cmd_vel_sub = rospy.Subscriber(TOPIC_robot_cmd_vel, Twist, self.callback)
        rospy.spin()

    def callback(self, msg):
        if (self.started == False) and (msg.linear.x != 0 or msg.linear.y != 0):
            self.started = True
            self.run_scenario()

    def run_scenario(self):
        f = 10
        rate = rospy.Rate(f)

        x0_r = 0.0
        y0_r = 0.0
        v_r = 0.6

        x0_h = self.x0_h
        y0_h = self.y0_h
        v_h = self.v_h
        d = self.d

        v = Twist()
        v.linear = Vector3(0,0,0)
        v.angular = Vector3(0,0,0)

        # All the known times
        t1 = (x0_h - d) / (v_r + v_h)
        x1 = x0_h - v_h*t1
        y1 = 0.0
        th1 = RAD(225)

        t2 = t1 + math.sqrt(2) / v_h
        x2 = x1 - 1
        y2 = -1

        x3 = 0.0
        y3 = 0.0
        deltax = x3 - x2
        deltay = y3 - y2
        t3 = t2 + (norm(np.array([deltax,deltay]))) / v_h
        th2 = math.atan2(deltay, deltax)

        p = Pose()
        p.position = Point(x0_h, y0_h, 0)
        q = tf.transformations.quaternion_from_euler(0,0,RAD(180))
        p.orientation = Quaternion(q[0],q[1],q[2],q[3])
        self.setpose_pub.publish(p)
        rate.sleep()

        i = 0
        while (i < t1*f):
            v.linear.x = v_h
            self.cmdvel_pub.publish(v)
            i += 1
            rate.sleep()

        p.position = Point(x1, y1, 0)
        q = tf.transformations.quaternion_from_euler(0,0,th1)
        p.orientation = Quaternion(q[0],q[1],q[2],q[3])
        self.setpose_pub.publish(p)
        rate.sleep()

        while (i < t2*f):
            v.linear.x = v_h
            self.cmdvel_pub.publish(v)
            i += 1
            rate.sleep()

        p.position = Point(x2, y2, 0)
        q = tf.transformations.quaternion_from_euler(0,0,th2)
        p.orientation = Quaternion(q[0],q[1],q[2],q[3])
        self.setpose_pub.publish(p)
        rate.sleep()

        while (i < t3*f):
            v.linear.x = v_h
            self.cmdvel_pub.publish(v)
            i += 1
            rate.sleep()

        p.position = Point(x3, y3, 0)
        q = tf.transformations.quaternion_from_euler(0,0,RAD(180))
        p.orientation = Quaternion(q[0],q[1],q[2],q[3])
        self.setpose_pub.publish(p)
        rate.sleep()

if __name__ == '__main__':

    if len(sys.argv) != 5:
        sys.exit(0)

    x0_h = float(sys.argv[1])
    y0_h = float(sys.argv[2])
    v_h = float(sys.argv[3])
    d = float(sys.argv[4])

    s1 = Scenario1(x0_h, y0_h, v_h, d)
    s1.subscribe()
