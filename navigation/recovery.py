import os,sys 

sys.path.append(os.getenv('MARRTINO_APPS_HOME')+'/program')

from robot_cmd_ros import *

TOPIC_planner_state = 'planner_state'



def do_recovery():
  setSpeed(-0.1,0,5.0)
  setSpeed(0.1,0,5.0)


recovery = False

def planner_state_cb(msg):
    global recovery
    if (msg.data!='running' and msg.data!='aborted'):
        print msg.data

    if 'recovery' in msg.data:
        recovery = True

begin()

odom_sub = rospy.Subscriber(TOPIC_planner_state, String, planner_state_cb)

enableObstacleAvoidance()

while marrtino_ok():
  if recovery:
    do_recovery()
    recovery = False
  time.sleep(0.5)

end()
