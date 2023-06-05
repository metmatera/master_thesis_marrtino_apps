import rospy, time, sys
import numpy as np
from numpy.linalg import norm

from geometry_msgs.msg import Twist


class Velocity(object):

    def __init__(self, scenario, filename):
        self.file = open("experiments/scenario"+scenario+"/test/cmdvel_"+filename+".txt", "w")
        self.file.write("# Robot Command Velocity - "+filename + "\n")
        self.started = False
        self.init_vel = 0.0
        self.t = 0.0
        self.file.write(str(self.t) + ',' + str(self.init_vel) + '\n')

    def CmdVelSub(self):
        rospy.init_node('Get_Cmd_Vel')

        cmdvel_sub = rospy.Subscriber("/cmd_vel", Twist, self.CmdVelCB)
        rospy.spin()

    def CmdVelCB(self, msg):

        vx = msg.linear.x
        vy = msg.linear.y
        v = np.array([vx,vy])
        vel = norm(v)

        if (self.started == False and vel != self.init_vel):
            self.started = True

        if (self.started == True):
            self.t += 0.1
            self.file.write(str(self.t) + "," + str(vel) + "\n")


if __name__ == '__main__':

    if (len(sys.argv) != 3):
        print("Missing arguments...")
        sys.exit(0)

    scenario = sys.argv[1]
    filename = sys.argv[2]

    velocity = Velocity(scenario, filename)
    velocity.CmdVelSub()
