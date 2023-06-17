import rospy, time, math

from people_msgs.msg import PositionMeasurementArray

def ptm_cb(msg):
    if len(msg.people) != 0:
        x = msg.people[0].pos.x
        y = msg.people[0].pos.y
        dist = math.sqrt(pow(x,2)+pow(y,2))
        print("Distance: " + str(dist))

if __name__ == '__main__':
    rospy.init_node('check_detection_position', disable_signals=True)

    ptm_sub = rospy.Subscriber('/leg_tracker_measurements', PositionMeasurementArray, ptm_cb)

    rospy.spin()
