import rospy, time, sys

from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegmentType, TrackedSegment, AgentType
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
import message_filters

import tf2_ros
import tf2_geometry_msgs

TOPIC_tracked_agents = '/tracked_agents'
TOPIC_tracker = '/people_tracker_measurements'

class AgentsBridge(object):

    def __init__(self):
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.Agent_State = TrackedAgent.MOVING
        self.Agent_Type = AgentType.HUMAN

    def AgentsPub(self):
        rospy.init_node('Agents_Bridge')
        rospy.sleep(1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tracked_agents_pub = rospy.Publisher(TOPIC_tracked_agents, TrackedAgents, queue_size=1)

        ptm_sub = rospy.Subscriber(TOPIC_tracker, PositionMeasurementArray, self.AgentsCB)

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
            target_frame = 'map'
            source_frame = agent.header.frame_id
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                           # get the tf at the time the pose was valid
                                           msg.header.stamp,
                                           # wait for transform, otherwise throw
                                           rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(1)
                continue

            agent_pose = PoseWithCovariance()
            agent_pose.pose.position = agent.pos

            map_pose = tf2_geometry_msgs.do_transform_pose(agent_pose, transform)
            agent_segment.pose.pose.position = map_pose.pose.position
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
            i += 1

        if (tracked_agents.agents):
            tracked_agents.header.stamp = msg.header.stamp # rospy.Time.now()
            tracked_agents.header.frame_id = 'map'
            self.tracked_agents_pub.publish(tracked_agents)
            #print("Published agents: " + str(len(tracked_agents.agents)))


if __name__ == '__main__':

    print("Running...")

    agents_bridge = AgentsBridge()
    agents_bridge.AgentsPub()
