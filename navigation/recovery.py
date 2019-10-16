import os,sys 

sys.path.append(os.getenv('MARRTINO_APPS_HOME')+'/program')

from robot_cmd_ros import *

TOPIC_planner_state = 'planner_state'

recovery = False  # when to do recovery


def do_recovery():
  enableObstacleAvoidance(True)
  setSpeed(-0.1,0,3.0)
  setSpeed(0.15,0,6.0)
  enableObstacleAvoidance(False)


def planner_state_cb(msg):
    global recovery
    if (msg.data!='running' and msg.data!='aborted'):
        #print msg.data
        pass
    if 'recovery' in msg.data:
        recovery = True

begin(use_desired_cmd_vel=True)

odom_sub = rospy.Subscriber(TOPIC_planner_state, String, planner_state_cb)

while marrtino_ok():
  if recovery:
    do_recovery()
    recovery = False
  time.sleep(0.5)

end()
