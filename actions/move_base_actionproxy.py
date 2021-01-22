# transform move_base string to move_base goal

import time, math, sys

import rospy
import tf

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionproxy import ActionProxy

ACTION_move_base = 'move_base'

class MoveBaseActionProxy(ActionProxy):

    def __init__(self):
        ActionProxy.__init__(self)
        nodename = 'move_base_actionproxy'
        rospy.init_node(nodename,  disable_signals=True)
        self.ac_movebase = None

    def send_goal(self, params):

        if self.ac_movebase == None:
            self.ac_movebase = actionlib.SimpleActionClient(ACTION_move_base,MoveBaseAction)

        self.ac_movebase.wait_for_server()

        v = params.split('_')
        gx = float(v[0])
        gy = float(v[1])
        gth_deg = float(v[2])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        yaw = gth_deg/180.0*math.pi
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        self.ac_movebase.send_goal(goal)

        rospy.loginfo("move_base: goal %s sent!" %(params))


    def monitor_thread(self, params): 

        self.send_goal(params)

        finished = False
        while self.do_run and not finished:
            self.ac_movebase.wait_for_result(rospy.Duration(1))
            finished = self.ac_movebase.get_state() == GoalStatus.SUCCEEDED

        state = self.ac_movebase.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("move_base: goal succeeded!")
        else:
            rospy.loginfo("move_base: goal failed!")

        self.ac_movebase.get_result()

        self.ac_movebase.cancel_all_goals()




if __name__ == "__main__":

    params = "0_0_0"
    if (len(sys.argv)>1):
        params = sys.argv[1]

    move = MoveBaseActionProxy()
    move.start(params)

    while (move.isRunning()):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            move.interrupt()
    
    move.end()


