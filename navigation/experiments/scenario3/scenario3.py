import math, time, sys, tf

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion

TOPIC_cmd_vel = '/human1/cmd_vel'
TOPIC_setpose = '/human1/setpose'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

if __name__ == '__main__':

    rospy.init_node('scenario3', disable_signals=True)
    f = 10
    rate = rospy.Rate(f)
    rate.sleep()

    if len(sys.argv) < 7:
        sys.exit(0)

    x0_r = float(sys.argv[1])
    y0_r = float(sys.argv[2])
    v_r = float(sys.argv[3])

    x0_h = float(sys.argv[4])
    y0_h = float(sys.argv[5])
    v_h = float(sys.argv[6])

    d1 = float(sys.argv[7])
    d2 = float(sys.argv[8])

    alpha = v_r / v_h
    l = (d1 + d2 + 2*math.sqrt(2)*alpha - 2) / (1 - alpha)

    cmdvel_pub = rospy.Publisher(TOPIC_cmd_vel, Twist, queue_size=10, latch=True)
    setpose_pub = rospy.Publisher(TOPIC_setpose, Pose, queue_size=1, latch=True)

    v = Twist()
    v.linear = Vector3(0,0,0)
    v.angular = Vector3(0,0,0)

    # All the known times
    t1 = (d1 + x0_h - x0_r) / (v_r - v_h)
    x1 = x0_h + v_h*t1
    y1 = 0.0

    t2 = t1 + math.sqrt(2) / v_h
    x2 = x1 + 1
    y2 = 1

    t3 = t2 + l / v_h
    x3 = x2 + l
    y3 = 1

    t4 = t3 + math.sqrt(2) / v_h
    x4 = x3 + 1
    y4 = 0

    t5 = t4 + 6.0 / v_h
    x5 = x4 + 6.0
    y5 = 0

    p = Pose()
    p.position = Point(x0_h, y0_h, 0)
    q = tf.transformations.quaternion_from_euler(0,0,0)
    p.orientation = Quaternion(q[0],q[1],q[2],q[3])
    setpose_pub.publish(p)
    rate.sleep()

    i = 0
    while (i < t1*f):
        v.linear.x = v_h
        cmdvel_pub.publish(v)
        i += 1
        rate.sleep()

    p.position = Point(x1, y1, 0)
    q = tf.transformations.quaternion_from_euler(0,0,RAD(45))
    p.orientation = Quaternion(q[0],q[1],q[2],q[3])
    setpose_pub.publish(p)
    rate.sleep()

    while (i < t2*f):
        v.linear.x = v_h
        cmdvel_pub.publish(v)
        i += 1
        rate.sleep()

    p.position = Point(x2, y2, 0)
    q = tf.transformations.quaternion_from_euler(0,0,0)
    p.orientation = Quaternion(q[0],q[1],q[2],q[3])
    setpose_pub.publish(p)
    rate.sleep()

    while (i < t3*f):
        v.linear.x = v_h
        cmdvel_pub.publish(v)
        i += 1
        rate.sleep()

    p.position = Point(x3, y3, 0)
    q = tf.transformations.quaternion_from_euler(0,0,RAD(-45))
    p.orientation = Quaternion(q[0],q[1],q[2],q[3])
    setpose_pub.publish(p)
    rate.sleep()

    while (i < t4*f):
        v.linear.x = v_h
        cmdvel_pub.publish(v)
        i += 1
        rate.sleep()

    p.position = Point(x4, y4, 0)
    q = tf.transformations.quaternion_from_euler(0,0,0)
    p.orientation = Quaternion(q[0],q[1],q[2],q[3])
    setpose_pub.publish(p)
    rate.sleep()

    while (i < t5*f):
        v.linear.x = v_h
        cmdvel_pub.publish(v)
        i += 1
        rate.sleep()

    p.position = Point(x5, y5, 0)
    q = tf.transformations.quaternion_from_euler(0,0,0)
    p.orientation = Quaternion(q[0],q[1],q[2],q[3])
    setpose_pub.publish(p)
    rate.sleep()

    print("\nQuit.")
