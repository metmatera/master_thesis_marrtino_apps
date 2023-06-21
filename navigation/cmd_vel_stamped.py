import rospy, time

from geometry_msgs.msg import Twist, TwistStamped

TOPIC_cmdvel = '/cmd_vel'
TOPIC_cmdvelstamped = '/cmd_vel_stamped'

class CmdVelStamped(object):

    def __init__(self):
        self.pub = rospy.Publisher(TOPIC_cmdvelstamped, TwistStamped, queue_size=1)

    def publisher(self):
        rospy.init_node('Cmd_Vel_Stamped')
        rospy.sleep(1)

        sub = rospy.Subscriber(TOPIC_cmdvel, Twist, self.callback)

        rospy.spin()

    def callback(self, msg):
        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = 'map'
        ts.twist = msg

        self.pub.publish(ts)


if __name__ == '__main__':

    print("Running...")

    cvs = CmdVelStamped()
    cvs.publisher()
