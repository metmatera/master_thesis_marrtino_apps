import rospy, time, sys, os

from cohan_msgs.msg import PointArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class Trajectory(object):

    def __init__(self, filename):
        self.trajectory = PointArray()
        self.traj_pub = rospy.Publisher("/trajectory", PointArray, queue_size=1)
        self.file = open("trajs/"+filename+".txt", "w")
        self.file.write("# "+filename + "\n")
        self.curr_x = 0.0
        self.curr_y = 0.0

    def TrajectoryPub(self):
        rospy.init_node('Trajectory')
        rospy.sleep(1)

        odom_sub = rospy.Subscriber("/odom", Odometry, self.TrajectoryCB)

        rospy.spin()

    def TrajectoryCB(self, msg):
        self.trajectory.header = msg.header
        self.trajectory.points.append(msg.pose.pose.position)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if (x != self.curr_x or y != self.curr_y):
            self.file.write(str(x) + "," + str(y) + "\n")
            self.curr_x = x
            self.curr_y = y

        if (self.trajectory):
            self.traj_pub.publish(self.trajectory)


if __name__ == '__main__':

    if (len(sys.argv) != 2):
        print "Missing filename parameter (type it without extension)"
        sys.exit(0)

    filename = sys.argv[1]

    trajectory = Trajectory(filename)
    trajectory.TrajectoryPub()
