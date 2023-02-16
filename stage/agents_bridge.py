import rospy, time, sys

from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegmentType, TrackedSegment
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import message_filters


class StageAgents(object):
	
	def __init__(self):
		self.tracked_agents_pub = []
		self.Segment_Type = TrackedSegmentType.TORSO
		self.n_agents = 0
		self.tmp_n_agents = self.n_agents
	
	def AgentsPub(self):
		rospy.init_node('Stage_Agents', anonymous=True)
		ptm_sub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.AgentsCB)
		self.tracked_agents_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=1)
		rospy.spin()
	
	def AgentsCB(self, msg):
		tracked_agents = TrackedAgents()
		for agent in msg.people:
			agent_segment = TrackedSegment()
			agent_segment.type = self.Segment_Type
			agent_segment.pose.pose = agent.pos
			tracked_agent = TrackedAgent()
			tracked_agent.track_id = agent.object_id
			tracked_agent.segments.append(agent_segment)
			tracked_agents.agents.append(tracked_agent)
			
		if (tracked_agents.agents):
			tracked_agents.header.stamp = rospy.Time.now()
			tracked_agents.header.frame_id = 'map'
			self.tracked_agents_pub.publish(tracked_agents)
			
		# Debug print
		#self.n_agents = len(tracked_agents.agents)
		#if (self.n_agents != self.tmp_n_agents):
		#	print "Number of tracked agents: " + str(self.n_agents)
		#	self.tmp_n_agents = self.n_agents

if __name__ == '__main__':
	agents = StageAgents()
	agents.AgentsPub()
