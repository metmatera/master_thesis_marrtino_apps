import rospy, time, sys

from cohan_msgs.msg import PointArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class Trajectory(object):

    def __init__(self):
        self.trajectory = PointArray()
        self.traj_pub = rospy.Publisher("/trajectory", PointArray, queue_size=1)

    def TrajectoryPub(self):
        rospy.init_node('Trajectory')
        rospy.sleep(1)

        odom_sub = rospy.Subscriber("/odom", Odometry, self.TrajectoryCB)

        rospy.spin()

    def TrajectoryCB(self, msg):
        self.trajectory.header = msg.header
        self.trajectory.points.append(msg.pose.pose.position)

        if (self.trajectory):
            self.traj_pub.publish(self.trajectory)

if __name__ == '__main__':

    trajectory = Trajectory()
    trajectory.TrajectoryPub()
