import rospy, time, sys

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
        self.set_start_time = False
        self.start_time = 0.0

    def TrajectoryPub(self):
        rospy.init_node('Trajectory')
        rospy.sleep(1)

        odom_sub = rospy.Subscriber("/odom", Odometry, self.TrajectoryCB)

        rospy.spin()

    def TrajectoryCB(self, msg):
        self.trajectory.header = msg.header
        self.trajectory.points.append(msg.pose.pose.position)

        if (self.set_start_time == False):
            self.start_time = time.time()
            self.set_start_time = True

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = time.time() - self.start_time
        self.file.write(str(t) + "," + str(x) + "," + str(y) + "\n")

        if (self.trajectory):
            self.traj_pub.publish(self.trajectory)


if __name__ == '__main__':

    if (len(sys.argv) != 2):
        print "Missing filename parameter (type it without extension)"
        sys.exit(0)

    filename = sys.argv[1]

    trajectory = Trajectory(filename)
    trajectory.TrajectoryPub()
