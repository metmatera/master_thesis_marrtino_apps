import rospy, time, sys

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CohanTrajectory(object):

    def __init__(self, filename, type):
        self.file = open("trajs/" + filename + "_cohan_" + type + ".txt", "w")
        self.file.write("# " + filename + " - " + type + " plan\n")
        self.plan = "/move_base/HATebLocalPlannerROS/" + type + "_plan"

    def getCohanTrajectory(self):
        rospy.init_node('Cohan_Trajectory')

        # This function subscribes one single message
        plan_msg = rospy.wait_for_message(self.plan, Path, timeout=20)

        poses = plan_msg.poses
        for p in poses:
            x = p.pose.position.x
            y = p.pose.position.y
            self.file.write(str(x) + "," + str(y) + "\n")

if __name__ == '__main__':

    if (len(sys.argv) != 3):
        print "Missing parameters:\n1) Filename without extension (example: t1)\n2) Plan type ('global' or 'local')"
        sys.exit(0)

    filename = sys.argv[1]
    type = sys.argv[2]

    trajectory = CohanTrajectory(filename, type)
    trajectory.getCohanTrajectory()
