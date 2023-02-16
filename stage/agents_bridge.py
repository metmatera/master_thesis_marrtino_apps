import rospy, time, sys

from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegmentType, TrackedSegment, AgentType
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import message_filters


class StageAgents(object):
	
	def __init__(self):
		self.tracked_agents_pub = []
		self.Segment_Type = TrackedSegmentType.TORSO
		self.Agent_State = TrackedAgent.MOVING
		self.Agent_Type = AgentType.HUMAN
	
	def AgentsPub(self):
		rospy.init_node('Stage_Agents', anonymous=True)
		ptm_sub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.AgentsCB)
		'''
		* std_msgs/Header
		
		uint32 	seq
		time 	stamp
		string 	frame_id
		
		* people_msgs/PositionMeasurementArray
		
		std_msgs/Header 			header
		people_msgs/PositionMeasurement[] 	people
		float32[] 				cooccurence
		
		* people_msgs/PositionMeasurement
		
		std_msgs/Header 	header
		string 			name
		geometry_msgs/Point 	pos
		float64 		reliability
		float64[9] 		covariance
		byte 			initialization
		'''
		
		self.tracked_agents_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=1)
		'''
		* cohan_msgs/TrackedAgents
		
		std_msgs/Header 		header
		cohan_msgs/TrackedAgent[] 	agents
		
		* cohan_msgs/TrackedAgent
		
		uint64 				track_id
		int8 				state
		int8 				type
		string 				name
		cohan_msgs/TrackedSegment[] 	segments
		
		* cohan_msgs/TrackedSegment
		
		int8 					type
		geometry_msgs/PoseWithCovariance 	pose
		geometry_msgs/TwistWithCovariance 	twist
		geometry_msgs/AccelWithCovariance 	accel
		'''
		
		rospy.spin()
	
	def AgentsCB(self, msg):
		tracked_agents = TrackedAgents()
		for agent in msg.people:
		
			tracked_agent = TrackedAgent()
			tracked_agent.state = self.Agent_State
			tracked_agent.type = self.Agent_Type
			tracked_agent.name = agent.name
			agent_segment = TrackedSegment()
			agent_segment.type = self.Segment_Type
			agent_segment.pose.pose.position = agent.pos
			tracked_agent.segments.append(agent_segment)
			
			tracked_agents.agents.append(tracked_agent)
			
		if (tracked_agents.agents):
			tracked_agents.header.stamp = rospy.Time.now()
			tracked_agents.header.frame_id = 'map'
			self.tracked_agents_pub.publish(tracked_agents)
			

if __name__ == '__main__':
	agents = StageAgents()
	agents.AgentsPub()
