import sys
import numpy as np
import rospy
import tf

from geometry_msgs.msg import Pose

TOPIC_setpose = '/setpose'

def DEG(a):
    return a*180.0/np.pi

def RAD(a):
    return a*np.pi/180.0

def pose_str(p):
	if p is None:
		return ""
    	else:
    		return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))
	
class PathFinderController:
	def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
		self.Kp_rho = Kp_rho
		self.Kp_alpha = Kp_alpha
		self.Kp_beta = Kp_beta
	
	def compute_control_command(self, x_diff, y_diff, theta, theta_goal):
		rho = np.hypot(x_diff, y_diff)
		alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
		beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
		v = self.Kp_rho * rho
		w = self.Kp_alpha * alpha - self.Kp_beta * beta
		
		if alpha > np.pi/2 or alpha < -np.pi/2:
			v = -v
		
		return rho, v, w

# simulation parameters
KP_RHO = 9
KP_ALPHA = 15
KP_BETA = 3
controller = PathFinderController(KP_RHO, KP_ALPHA, KP_BETA)
dt = 0.01

# Constraints specifications
MAX_LINEAR_SPEED = 15
MAX_ANGULAR_SPEED = 7

def getTrajectory(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
	x = x_start
	y = y_start
	theta = theta_start

	x_diff = x_goal - x
	y_diff = y_goal - y

	x_traj, y_traj, theta_traj = [], [], []

	rho = np.hypot(x_diff, y_diff)
	while rho > 0.001:
		x_traj.append(x)
		y_traj.append(y)
		theta_traj.append(theta)

		x_diff = x_goal - x
		y_diff = y_goal - y

		rho, v, w = controller.compute_control_command(x_diff, y_diff, theta, theta_goal)

		if abs(v) > MAX_LINEAR_SPEED:
			v = np.sign(v) * MAX_LINEAR_SPEED

		if abs(w) > MAX_ANGULAR_SPEED:
			w = np.sign(w) * MAX_ANGULAR_SPEED

		theta = theta + w * dt
		x = x + v * np.cos(theta) * dt
		y = y + v * np.sin(theta) * dt
		
	return x_traj, y_traj, theta_traj

# main
if __name__ == "__main__":

	rospy.init_node('setstagepose', disable_signals=True)
	rate = rospy.Rate(10)

	if len(sys.argv) != 8:
		print("Error: %s <name> <x_i> <y_i> <theta_i> <x_d> <y_d> <theta_d>" %(sys.argv[0]))
		sys.exit(0)

	human = sys.argv[1]
	x_start = float(sys.argv[2])
	y_start = float(sys.argv[3])
	theta_start = RAD(float(sys.argv[4]))
	x_goal = float(sys.argv[5])
	y_goal= float(sys.argv[6])
	theta_goal = RAD(float(sys.argv[7]))
	
	print("%s goes from [%.2f %.2f %.2f RAD] to [%.2f %.2f %.2f RAD]" %(human, x_start, y_start, theta_start, x_goal, y_goal, theta_goal))
	
	print("Computing the trajectory...")
	x_traj, y_traj, theta_traj = getTrajectory(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)
	print("Done!")

	# Publisher for /human{i}/setpose
	TOPIC_setpose = "/" + human + TOPIC_setpose
	setpose_pub = rospy.Publisher(TOPIC_setpose, Pose, queue_size=10, latch=True)
	
	# Initial configuration
	p = Pose()
	p.position.x = x_start
	p.position.y = y_start
	p.position.z = 0.0
	q = tf.transformations.quaternion_from_euler(0,0,theta_start)
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	
	setpose_pub.publish(p)
	rate.sleep()
	
	#XXX Debug
	#assert len(x_traj) == len(y_traj)
	#assert len(y_traj) == len(theta_traj)
	
	print("Moving...")
	timesteps = len(x_traj)
	for i in range(timesteps):
		p.position.x = x_traj[i]
		p.position.y = y_traj[i]
		q = tf.transformations.quaternion_from_euler(0,0,theta_traj[i])
		p.orientation.x = q[0]
		p.orientation.y = q[1]
		p.orientation.z = q[2]
		p.orientation.w = q[3]
		
		setpose_pub.publish(p)
		rate.sleep()
		
		
	q = tf.transformations.quaternion_from_euler(0,0,theta_goal)
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]	
	setpose_pub.publish(p)
	print("Goal reached!")
	
	
	
	
	

