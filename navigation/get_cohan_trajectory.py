import rospy, time, sys

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CohanTrajectory(object):

    def __init__(self, filename):
        self.file = open("trajs/"+filename+"_cohan.txt", "w")
        self.file.write("# " + filename + "\n")

    def getCohanTrajectory(self):
        rospy.init_node('CohanTrajectory')
        #rospy.sleep(1)

        plan_msg = rospy.wait_for_message("/move_base/GlobalPlanner/plan", Path, timeout=5)

        poses = plan_msg.poses
        t = 0.0
        for p in poses:
            x = p.pose.position.x
            y = p.pose.position.y
            self.file.write(str(t) + "," + str(x) + "," + str(y) + "\n")
            t += 0.1

if __name__ == '__main__':

    if (len(sys.argv) != 2):
        print "Missing filename parameter (type it without extension)"
        sys.exit(0)

    filename = sys.argv[1]

    trajectory = CohanTrajectory(filename)
    trajectory.getCohanTrajectory()
