import rospy, time, sys

import message_filters
import numpy as np
from numpy.linalg import norm

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

TOPIC_robot_pose = '/base_pose_ground_truth'
TOPIC_human_pose = '/human1/base_pose_ground_truth'
TOPIC_cmdvel = '/cmd_vel_stamped'

class Path(object):

    def __init__(self, scenario, filename, robot_init_pos, human_vel):
        path = 'experiments/scenario'+scenario+'/test/'+filename+'.txt'
        self.file = open(path, 'w')
        self.file.write('# Scenario: ' + scenario + '\n')
        self.file.write('# t,x_r,y_r,v_r,x_h,y_h,v_h\n')
        self.started = False
        self.robot_init_pos = robot_init_pos
        self.t = 0.0
        self.v_h = human_vel

    def PathSub(self):
        rospy.init_node('Paths_Writer')
        rospy.sleep(1)

        robot_pos_sub = message_filters.Subscriber(TOPIC_robot_pose, Odometry)
        human_pos_sub = message_filters.Subscriber(TOPIC_human_pose, Odometry)
        vel_sub = message_filters.Subscriber(TOPIC_cmdvel, TwistStamped)
        ts = message_filters.TimeSynchronizer([robot_pos_sub, human_pos_sub, vel_sub], 10)
        ts.registerCallback(self.PathCB)
        rospy.spin()

    def PathCB(self, robot_msg, human_msg, vel_msg):

        # Robot info
        x_r = robot_msg.pose.pose.position.x
        y_r = robot_msg.pose.pose.position.y
        vx = vel_msg.twist.linear.x
        vy = vel_msg.twist.linear.y
        v_r = norm(np.array([vx,vy]))

        # Human info
        x_h = human_msg.pose.pose.position.x
        y_h = human_msg.pose.pose.position.y
        v_h = self.v_h

        if self.started == False and v_r != 0.0:
            x0_r = self.robot_init_pos[0]
            y0_r = self.robot_init_pos[1]
            v0_r = 0.0
            v0_h = 0.0
            self.file.write(str(self.t)+','+str(x0_r)+','+str(y0_r)+','+str(v0_r)+','+str(x_h)+','+str(y_h)+','+str(v0_h)+'\n')
            self.t += 0.1   # Time rate 10 Hz
            self.file.write(str(self.t)+','+str(x_r)+','+str(y_r)+','+str(v_r)+','+str(x_h)+','+str(y_h)+','+str(v_h)+'\n')
            self.started = True

        elif self.started:
            self.t += 0.1
            self.file.write(str(self.t)+','+str(x_r)+','+str(y_r)+','+str(v_r)+','+str(x_h)+','+str(y_h)+','+str(v_h)+'\n')

if __name__ == '__main__':

    if (len(sys.argv) != 6):
        print("Missing arguments...")
        sys.exit(0)

    scenario = sys.argv[1]
    filename = sys.argv[2]
    x0_r = float(sys.argv[3])
    y0_r = float(sys.argv[4])
    p0_r = [x0_r, y0_r]
    v_h = float(sys.argv[5])

    path = Path(scenario, filename, p0_r, v_h)
    path.PathSub()
