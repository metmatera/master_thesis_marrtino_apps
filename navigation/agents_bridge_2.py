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

    def is_ready(self, message):
        cnt = 0
        for msg in message:
            if msg.ns == "LEGS":
                cnt += 1
        if cnt == 2:
            return True
        else:
            return False

    def AgentsPub(self):
        rospy.init_node('Agents_Bridge')
        rospy.sleep(1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tracked_agents_pub = rospy.Publisher(self.topic_pub, TrackedAgents, queue_size=1)
        sub = rospy.Subscriber(self.topic_sub, Marker, self.AgentsCB)
        rospy.spin()

    def AgentsCB(self, msg):
        self.message.append(msg)
        if self.is_ready(self.message):
            left_leg = self.message[0]
            right_leg = self.message[-1]
            agent = None
            if len(self.message) == 3:
                agent = self.message[1]
            else:
                agent = Marker()
                agent = left_leg
                agent.ns = "PEOPLE"
                agent.id = 0
                agent.pose.position.x = (left_leg.pose.position.x + right_leg.pose.position.x) / 2
                agent.pose.position.y = (left_leg.pose.position.y + right_leg.pose.position.y) / 2
                agent.pose.position.z = 1.70

            tracked_agents = TrackedAgents()
            tracked_agents.header.stamp = msg.header.stamp
            tracked_agents.header.frame_id = 'map'
            tracked_agent = TrackedAgent()
            tracked_agent.track_id = 1
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

            '''
            # Segment for left leg of the agent
            left_leg_segment = TrackedSegment()
            left_leg_segment.type = TrackedSegmentType.LEFT_ANKLE
            left_leg_src_pose = PoseWithCovariance()
            left_leg_src_pose.pose = left_leg.pose
            left_leg_pose = tf2_geometry_msgs.do_transform_pose(left_leg_src_pose, transform)
            left_leg_segment.pose.pose.position = left_leg_pose.pose.position
            left_leg_segment.pose.pose.orientation = left_leg.pose.orientation
            tracked_agent.segments.append(left_leg_segment)

            # Segment for right leg of the agent
            right_leg_segment = TrackedSegment()
            right_leg_segment.type = TrackedSegmentType.RIGHT_ANKLE
            right_leg_src_pose = PoseWithCovariance()
            right_leg_src_pose.pose = right_leg.pose
            right_leg_pose = tf2_geometry_msgs.do_transform_pose(right_leg_src_pose, transform)
            right_leg_segment.pose.pose.position = right_leg_pose.pose.position
            right_leg_segment.pose.pose.orientation = right_leg.pose.orientation
            tracked_agent.segments.append(right_leg_segment)
            '''

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
            self.tracked_agents_pub.publish(tracked_agents)
            self.message = []

if __name__ == '__main__':

    print("Running...")

    agents_bridge = AgentsBridge()
    agents_bridge.AgentsPub()
