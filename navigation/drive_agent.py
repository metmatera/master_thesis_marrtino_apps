import math, sys, time
import rospy, tf
from getch import getch

from geometry_msgs.msg import Pose

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

def get_key():
    first_char = getch()
    if first_char == '\x1b':
        return {'[A': 'up', '[B': 'down', '[C': 'right', '[D': 'left'}[getch() + getch()]
    else:
        return first_char

def move(pose, cmd):
    if (cmd == 'right'):
        pose.position.x += 0.02
    elif (cmd == 'left'):
        pose.position.x -= 0.02
    elif (cmd == 'up'):
        pose.position.y += 0.02
    elif (cmd == 'down'):
        pose.position.y -= 0.02
    elif (cmd == 'q'):
        print("Quit.")
    else:
        print("Command not found!")

def rotate(pose, cmd, theta):
    if (cmd == 'l'):
        theta += 5
        q = tf.transformations.quaternion_from_euler(0,0,RAD(theta))
    elif (cmd == 'r'):
        theta -= 5
        q = tf.transformations.quaternion_from_euler(0,0,RAD(theta))
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return theta

# main
if __name__ == "__main__":

    rospy.init_node('drive_agent', disable_signals=True)
    rate = rospy.Rate(10)
    rate.sleep()

    if len(sys.argv) < 5:
        sys.exit(0)

    agent = sys.argv[1]
    TOPIC_setpose = '/' + agent + '/setpose'
    setpose_pub = rospy.Publisher(TOPIC_setpose, Pose, queue_size=1, latch=True)

    x = float(sys.argv[2])
    y = float(sys.argv[3])
    th = float(sys.argv[4])
    th_curr = th
    q = tf.transformations.quaternion_from_euler(0,0,RAD(th))

    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = 0
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]

    key = ''
    print("Ready to go!\nUse the keyboard arrows to move " + agent + " into the map...")
    while (key != 'q'):
        key = get_key()
        if key not in ['r', 'l']:
            move(p, key)
        else:
            th_curr = rotate(p, key, th_curr)

        setpose_pub.publish(p)
        #rate.sleep()
