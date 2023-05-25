import rospy, time, sys

from nav_msgs.msg import Odometry

class Trajectory(object):

    def __init__(self, scenario, filename):
        self.file = open("experiments/scenario"+scenario+"/test/"+filename+".txt", "w")
        self.file.write("# "+filename + "\n")
        self.set_init_pos = False
        self.init_pos = [0.0, 0.0]
        self.t = 0.0

    def TrajectorySub(self):
        rospy.init_node('Trajectory')

        odom_sub = rospy.Subscriber("/odom", Odometry, self.TrajectoryCB)
        rospy.spin()

    def TrajectoryCB(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if (self.set_init_pos == False):
            self.init_pos[0] = x
            self.init_pos[1] = y
            self.file.write(str(self.t) + "," + str(x) + "," + str(y) + "\n")
            self.set_init_pos = True

        # file not modified until the position is different from the initial one
        if (x != self.init_pos[0] or y != self.init_pos[1]):
            self.t += 0.1   # Rate of the Subscriber is 10 Hz
            self.file.write(str(self.t) + "," + str(x) + "," + str(y) + "\n")


if __name__ == '__main__':

    if (len(sys.argv) != 3):
        sys.exit(0)

    scenario = sys.argv[1]
    filename = sys.argv[2]

    trajectory = Trajectory(scenario, filename)
    trajectory.TrajectorySub()
