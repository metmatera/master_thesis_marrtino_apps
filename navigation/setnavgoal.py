import sys, rospy, time, math
import tf

from geometry_msgs.msg import PoseStamped, Point, Quaternion

TOPIC_navgoal = '/move_base_simple/goal'

def DEG(a):
	return a*180.0/math.pi

def RAD(a):
	return a*math.pi/180.0

# main
if __name__ == "__main__":

	rospy.init_node('setnavgoal', disable_signals=True)
	rospy.sleep(1)

	if len(sys.argv) != 4:
		print("%s <x> <y> <theta>" %(sys.argv[0]))
		sys.exit(0)

	x = float(sys.argv[1])
	y = float(sys.argv[2])
	theta = float(sys.argv[3])

	navgoal_pub = rospy.Publisher(TOPIC_navgoal, PoseStamped, queue_size=1)
	goal = PoseStamped()
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = 'map'
	goal.pose.position = Point(x,y,0)
	q = tf.transformations.quaternion_from_euler(0,0,RAD(theta))
	goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
	
	navgoal_pub.publish(goal)
	rospy.spin()
