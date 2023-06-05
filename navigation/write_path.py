import rospy, time, sys

from nav_msgs.msg import Odometry

class Path(object):

    def __init__(self, scenario, filename, init_pos):
        self.file = open("experiments/scenario"+scenario+"/test/path_"+filename+".txt", "w")
        self.file.write("# Robot - " + filename + "\n")
        self.started = False
        self.init_pos = init_pos
        self.t = 0.0

    def PathSub(self):
        rospy.init_node('Path')

        robot_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.PathCB)

        rospy.spin()

    def PathCB(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if (self.started == False):
            self.init_pos[0] = x
            self.init_pos[1] = y
            self.file.write(str(self.t) + "," + str(x) + "," + str(y) + "\n")
            self.started = True

        # file not modified until the position is different from the initial one
        elif (x != self.init_pos[0] or y != self.init_pos[1]):
            self.t += 0.1   # Rate of the Subscriber is 10 Hz
            self.file.write(str(self.t) + "," + str(x) + "," + str(y) + "\n")


if __name__ == '__main__':

    if (len(sys.argv) != 5):
        print("Missing arguments...")
        sys.exit(0)

    scenario = sys.argv[1]
    filename = sys.argv[2]
    x0_r = float(sys.argv[3])
    y0_r = float(sys.argv[4])
    p0_r = [x0_r, y0_r]

    path = Path(scenario, filename, p0_r)
    path.PathSub()
