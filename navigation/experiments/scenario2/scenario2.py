import rospy
import time
import sys
import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

TOPIC_robot_pose = '/base_pose_ground_truth'
TOPIC_human_pose = '/human1/base_pose_ground_truth'
TOPIC_human_cmd_vel = '/human1/cmd_vel'

class Scenario2(object):

    def __init__(self, v_h, d):
        self.v_h = v_h
        self.v = Twist()
        self.v.linear = Vector3(0,0,0)
        self.v.angular = Vector3(0,0,0)
        self.d = d
        self.cmdvel_pub = rospy.Publisher(TOPIC_human_cmd_vel, Twist, queue_size=10, latch=True)

    def run(self):
        rospy.init_node('Scenario2')
        rospy.sleep(1)

        robot_pose_sub = message_filters.Subscriber(TOPIC_robot_pose, Odometry)
        human_pose_sub = message_filters.Subscriber(TOPIC_human_pose, Odometry)
        ts = message_filters.TimeSynchronizer([robot_pose_sub, human_pose_sub], 10)
        ts.registerCallback(self.callback)
        rospy.spin()

    def callback(self, robot_msg, human_msg):
        xr = robot_msg.pose.pose.position.x
        yr = robot_msg.pose.pose.position.y
        xh = human_msg.pose.pose.position.x
        yh = human_msg.pose.pose.position.y
        if (xr != 0 or yr != 0) and (xr - xh < self.d):
            self.v.linear.x = self.v_h
        else:
            self.v.linear.x = 0
        self.cmdvel_pub.publish(self.v)


if __name__ == '__main__':

    if len(sys.argv) != 3:
        sys.exit(0)

    v_h = float(sys.argv[1])
    d = float(sys.argv[2])

    s1 = Scenario2(v_h, d)
    s1.run()
