import rospy, time, sys

from nav_msgs.msg import Odometry

class Trajectory(object):

    def __init__(self, filename):
        self.file = open("trajs/"+filename+".txt", "w")
        self.file.write("# "+filename + "\n")
        self.set_init_pose = False
        self.init_pose = [0.0, 0.0]
        self.t = 0.0

    def TrajectorySub(self):
        rospy.init_node('Trajectory')
        #rospy.sleep(1)
        odom_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.TrajectoryCB)
        rospy.spin()

    def TrajectoryCB(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if (self.set_init_pose == False):
            self.init_pose[0] = x
            self.init_pose[1] = y
            self.file.write(str(self.t) + "," + str(x) + "," + str(y) + "\n")
            self.set_init_pose = True

        if (x != self.init_pose[0] or y != self.init_pose[1]):
            self.t += 0.1
            self.file.write(str(self.t) + "," + str(x) + "," + str(y) + "\n")


if __name__ == '__main__':

    if (len(sys.argv) != 2):
        print "Missing filename parameter (type it without extension)"
        sys.exit(0)

    filename = sys.argv[1]

    trajectory = Trajectory(filename)
    trajectory.TrajectorySub()
