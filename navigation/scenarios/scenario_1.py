# SCENARIO 1:
# Map: DIAG-B1
# Robot encounters a human from the opposite direction in a corridor
# Robot starts at (0, 0, 0) and arrives to (11.26, 0, 0)
# Human starts at (16, 0, 180) and arrives to (4.15, 0, 180)
# Robot detects human at (6, 0) at starts turning left to avoid him
# Human always goes straight forward
# ROBOT TARGET GOAL: (11.26, 0, 0)

import rospy, math, tf, sys

from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point

TOPIC_cmd_vel = 'cmd_vel'

def DEG(a):
    return a*180.0/math.pi

def RAD(a):
    return a*math.pi/180.0

if __name__ == '__main__':

    rospy.init_node('scenario1', disable_signals=True)
    rate = rospy.Rate(10)
    rate.sleep()

    # Initializing the path structure
    robot_pub = rospy.Publisher(TOPIC_cmd_vel, Twist, queue_size=10, latch=True)

    robot_vel = Twist()
    robot_vel.linear = Vector3(0.0, 0.0, 0.0)
    robot_vel.angular = Vector3(0.0, 0.0, 0.0)

    i = 0
    while (i < 57):
        robot_vel.linear.x = 0.7
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = math.pi / 4
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 15):
        robot_vel.angular.z = 0
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = -math.pi / 4
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 15):
        robot_vel.angular.z = 0
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = -math.pi / 4
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 15):
        robot_vel.angular.z = 0
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 10):
        robot_vel.angular.z = math.pi / 4
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    i = 0
    while (i < 40):
        robot_vel.angular.z = 0
        robot_pub.publish(robot_vel)
        rate.sleep()
        i += 1

    print("Scenario 1 completed.")
    rospy.sleep(3)
