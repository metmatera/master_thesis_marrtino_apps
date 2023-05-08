# SCENARIO 1: Robot encounters a human from the opposite direction
# Robot starts at (3, 13, 0)
# Human starts at (19, 13, 180) and arrives to (7.15, 13, 180)
# Robot detects human at (9, 13) at starts turning left to avoid him
# Human always goes straight forward
# TARGET GOAL: (14.26, 13, 0)

import rospy, math, tf, sys

from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point

TOPIC_robot_cmd_vel = 'cmd_vel'
TOPIC_human_cmd_vel = '/human1/cmd_vel'
TOPIC_robot_setpose = '/setpose'
TOPIC_human_setpose = '/human1/setpose'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

if __name__ == '__main__':

    rospy.init_node('scenario1', disable_signals=True)
    rate = rospy.Rate(10)
    rate.sleep()

    # Setting initial poses for robot and human
    print("Initializing scenario...")
    robot_setpose_pub = rospy.Publisher(TOPIC_robot_setpose, Pose, queue_size=1, latch=True)
    human_setpose_pub = rospy.Publisher(TOPIC_human_setpose, Pose, queue_size=1, latch=True)

    robot_pose = Pose()
    robot_pose.position = Point(3, 13, 0)
    q = tf.transformations.quaternion_from_euler(0,0,RAD(0))
    robot_pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    human_pose = Pose()
    human_pose.position = Point(19, 13, 0)
    q = tf.transformations.quaternion_from_euler(0,0,RAD(180))
    human_pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    robot_setpose_pub.publish(robot_pose)
    human_setpose_pub.publish(human_pose)

    print("Initialized.")

    rospy.sleep(1)

    # Initializing the path structure
    robot_pub = rospy.Publisher(TOPIC_robot_cmd_vel, Twist, queue_size=10, latch=True)
    human_pub = rospy.Publisher(TOPIC_human_cmd_vel, Twist, queue_size=10, latch=True)

    robot_vel = Twist()
    robot_vel.linear = Vector3(0.0, 0.0, 0.0)
    robot_vel.angular = Vector3(0.0, 0.0, 0.0)

    human_vel = Twist()
    human_vel.linear = Vector3(0.0, 0.0, 0.0)
    human_vel.angular = Vector3(0.0, 0.0, 0.0)

    key = raw_input("Press T+Enter to test the robot on this scenario.\nOtherwise, press Enter to see the reference scenario.\n")

    if (key == 't' or key == 'T'):
        i = 0
        while (i < 234):
            human_vel.linear.x = 0.5

            human_pub.publish(human_vel)

            rate.sleep()
            i += 1
        print("Scenario 1 completed.")
        rospy.sleep(3)
        sys.exit(0)

    print("Scenario 1 started.")

    i = 0
    while (i < 120):
        robot_vel.linear.x = 0.5
        human_vel.linear.x = 0.5

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = math.pi / 4

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 15):
        robot_vel.angular.z = 0

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = -math.pi / 4

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 15):
        robot_vel.angular.z = 0

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = -math.pi / 4

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 15):
        robot_vel.angular.z = 0

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = math.pi / 4

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    i = 0
    while (i < 30):
        robot_vel.angular.z = 0

        robot_pub.publish(robot_vel)
        human_pub.publish(human_vel)

        rate.sleep()
        i += 1

    print("Scenario 1 completed.")
    rospy.sleep(3)
