import rospy, time, sys

from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegmentType, TrackedSegment, AgentType
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
import message_filters

import tf2_ros
import tf2_geometry_msgs

def have_last(agents, last_agents):
    if agents and last_agents:
        return True
    return False

def predict_delta(agent, last_agent):
    xf = agent.segments[0].pose.pose.position.x
    xi = last_agent.segments[0].pose.pose.position.x
    deltax = xf - xi
    yf = agent.segments[0].pose.pose.position.y
    yi = last_agent.segments[0].pose.pose.position.y
    deltay = yf - yi
    return [deltax, deltay]


class AgentsBridge(object):

    def __init__(self):
        self.Segment_Type = TrackedSegmentType.TORSO
        self.Agent_State = TrackedAgent.MOVING
        self.Agent_Type = AgentType.HUMAN
        self.last_tracked_agents = None
        self.tracked_agents = None

    def AgentsPub(self):
        rospy.init_node('Agents_Bridge')
        rospy.sleep(1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tracked_agents_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=1)

        leg_sub = message_filters.Subscriber('/leg_tracker_measurements', PositionMeasurementArray)
        people_sub = message_filters.Subscriber('/people_tracker_measurements', PositionMeasurementArray)
        ts = message_filters.TimeSynchronizer([leg_sub, people_sub], 10)
        ts.registerCallback(self.AgentsCB)
        rospy.spin()

    def AgentsCB(self, leg_msg, people_msg):
        tracked_agents = TrackedAgents()
        if len(people_msg.people) != 0:
            people = people_msg.people
            header = people_msg.header
        else:
            people = leg_msg.people
            header = leg_msg.header

        if len(people) == 0:
            tracked_agents = self.tracked_agents
            if have_last(self.tracked_agents, self.last_tracked_agents):
                for i in range(max(len(self.tracked_agents.agents), len(self.last_tracked_agents.agents))):
                    agent = self.tracked_agents.agents[i]
                    last_agent = self.last_tracked_agents.agents[i]
                    delta = predict_delta(agent, last_agent)
                    deltax, deltay = delta[0], delta[1]
                    tracked_agents.agents[i].segments[0].pose.pose.position.x += deltax
                    tracked_agents.agents[i].segments[0].pose.pose.position.y += deltay

        i = 1
        for agent in people:
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
                                           header.stamp,
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

        if tracked_agents != None and tracked_agents.agents:
            tracked_agents.header.stamp = header.stamp
            tracked_agents.header.frame_id = 'map'
            self.last_tracked_agents = self.tracked_agents
            self.tracked_agents = tracked_agents
            self.tracked_agents_pub.publish(tracked_agents)

if __name__ == '__main__':

    print("Running...")

    agents_bridge = AgentsBridge()
    agents_bridge.AgentsPub()
