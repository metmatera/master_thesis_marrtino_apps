import rospy, time, sys

from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegmentType, TrackedSegment, AgentType
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import message_filters


class StageAgents(object):
	
	def __init__(self, leg_detector=True, num_hum=0):
		self.tracked_agents_pub = []
		self.Segment_Type = TrackedSegmentType.TORSO
		self.Agent_State = TrackedAgent.MOVING
		self.Agent_Type = AgentType.HUMAN
		self.leg_detector = leg_detector
		self.num_hum = num_hum
	
	def AgentsPub(self):
		rospy.init_node('Stage_Agents', anonymous=True)
		
		self.tracked_agents_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=1)
		
		if (self.leg_detector):
			ptm_sub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.AgentsCB)
		
		else:
			hum_marker_sub = []
			for human_id in range(1,self.num_hum+1):
				name = 'human'+str(human_id)
				hum_marker_sub.append(message_filters.Subscriber("/" + name + "/base_pose_ground_truth", Odometry))
			pose_msg = message_filters.TimeSynchronizer(hum_marker_sub, 10)
			pose_msg.registerCallback(self.HumansCB)
		
		rospy.spin()
	
	def AgentsCB(self, msg):
		tracked_agents = TrackedAgents()
		i = 1
		for agent in msg.people:
			
			tracked_agent = TrackedAgent()
			tracked_agent.track_id = i
			tracked_agent.state = self.Agent_State
			tracked_agent.type = self.Agent_Type
			tracked_agent.name = agent.name
			agent_segment = TrackedSegment()
			agent_segment.type = self.Segment_Type
			agent_segment.pose.pose.position = agent.pos
			tracked_agent.segments.append(agent_segment)
			
			tracked_agents.agents.append(tracked_agent)
			i += 1
			
		if (tracked_agents.agents):
			tracked_agents.header.stamp = rospy.Time.now()
			tracked_agents.header.frame_id = 'map'
			self.tracked_agents_pub.publish(tracked_agents)
			
	def HumansCB(self,*msg):
		tracked_agents = TrackedAgents()
		for human_id in range(1,self.num_hum+1):
        	
			tracked_agent = TrackedAgent()
			tracked_agent.track_id = human_id

			agent_segment = TrackedSegment()
			agent_segment.type = self.Segment_Type
			agent_segment.pose.pose = msg[human_id-1].pose.pose
			agent_segment.twist.twist = msg[human_id-1].twist.twist

			tracked_agent.segments.append(agent_segment)
			tracked_agents.agents.append(tracked_agent)
            
		if (tracked_agents.agents):
			tracked_agents.header.stamp = rospy.Time.now()
			tracked_agents.header.frame_id = 'map'
			self.tracked_agents_pub.publish(tracked_agents)

if __name__ == '__main__':
	
	if (len(sys.argv) != 3):
		print("Missing parameters...\nleg_detector (int) 0|1:\n\t0 -> off\n\t1 -> on\nnum_hum (int) >=0:\n\tnumber of humans in the simulation")
	
	flag = int(sys.argv[1])
	
	leg_detector = True
	if (flag == 0):
		leg_detector = False
	elif (flag > 1):
		print("The value must be 0 or 1")
		sys.exit(0)

	num_hum = int(sys.argv[2])
	
	agents = StageAgents(leg_detector=leg_detector, num_hum=num_hum)	
	agents.AgentsPub()
	
	
	
	
	
	
	
	
