import rospy, time

from visualization_msgs.msg import Marker
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegmentType, TrackedSegment, AgentType
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
import message_filters

import tf2_ros
import tf2_geometry_msgs

class AgentsBridge(object):

    def __init__(self):
        self.topic_pub = '/tracked_agents'
        self.topic_sub = '/leg_marker'
        self.message = []

    def count_agent_detected(self, message):
        cnt = 0
        for msg in message:
            if msg.ns == "LEGS":
                cnt += 1
        return int(cnt/2)

    def get_people_idxs(self, n_people):
        idxs = []
        for i in range(n_people):
            idxs.append(1 + 3*i)
        return idxs

    def AgentsPub(self):
        rospy.init_node('Agents_Bridge')
        rospy.sleep(1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tracked_agents_pub = rospy.Publisher(self.topic_pub, TrackedAgents, queue_size=1)
        sub = rospy.Subscriber(self.topic_sub, Marker, self.AgentsCB)
        rospy.spin()

    def AgentsCB(self, msg):
        if msg.id != 0:
            self.message.append(msg)
        elif msg.ns == "LEGS" and msg.id == 0 and len(self.message) > 0:
            n_agents = self.count_agent_detected(self.message)
            agents_idxs = self.get_people_idxs(n_agents)
            agent_id = 0
            for i in agents_idxs:
                if self.message[i].ns != "PEOPLE":
                    left_leg = self.message[i-1]
                    right_leg = self.message[i]
                    agent = left_leg
                    agent.ns = "PEOPLE"
                    agent.id = agent_id
                    agent.pose.position.x = (left_leg.pose.position.x + right_leg.pose.position.x) / 2
                    agent.pose.position.y = (left_leg.pose.position.y + right_leg.pose.position.y) / 2
                    agent.pose.position.z = 1.70
                    self.message.insert(i, agent)
                else:
                    self.message[i].id = agent_id
                agent_id += 1

            tracked_agents = TrackedAgents()
            i = 1
            for i in agents_idxs:
                agent = self.message[i]
                tracked_agent = TrackedAgent()
                tracked_agent.track_id = i
                tracked_agent.state = TrackedAgent.MOVING
                tracked_agent.type = AgentType.HUMAN
                tracked_agent.name = 'human' + str(tracked_agent.track_id)

                source_frame = 'odom'
                target_frame = 'map'
                try:
                    transform = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                                msg.header.stamp,
                                                                rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('Lookup/Connectivity/Extrapolation Exception tracked')

                # Segment for center of the person
                agent_segment = TrackedSegment()
                agent_segment.type = TrackedSegmentType.HEAD
                agent_src_pose = PoseWithCovariance()
                agent_src_pose.pose = agent.pose
                agent_pose = tf2_geometry_msgs.do_transform_pose(agent_src_pose, transform)
                agent_segment.pose.pose.position = agent_pose.pose.position
                agent_segment.pose.pose.orientation = agent.pose.orientation
                tracked_agent.segments.append(agent_segment)
                tracked_agents.agents.append(tracked_agent)
                i += 1
            if (tracked_agents.agents):
                tracked_agents.header.stamp = msg.header.stamp
                tracked_agents.header.frame_id = 'map'
                self.tracked_agents_pub.publish(tracked_agents)

            # Reset message to empty
            self.message = []
            self.message.append(msg)

if __name__ == '__main__':

    print("Running...")

    agents_bridge = AgentsBridge()
    agents_bridge.AgentsPub()
