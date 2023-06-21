import rospy, time, math, sys

from people_msgs.msg import PositionMeasurementArray

TOPIC_people_tracker = '/people_tracker_measurements'
TOPIC_leg_tracker = '/leg_tracker_measurements'

def tracker_cb(msg):
    if len(msg.people) != 0:
        x = msg.people[0].pos.x
        y = msg.people[0].pos.y
        dist = math.sqrt(pow(x,2)+pow(y,2))
        print("Distance: " + str(dist))

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Missing argument: 'people' or 'leg'")
        sys.exit(0)
    rospy.init_node('check_detection_position', disable_signals=True)
    tracker = sys.argv[1]
    if tracker == 'people':
        sub = rospy.Subscriber(TOPIC_people_tracker, PositionMeasurementArray, tracker_cb)
    elif tracker == 'leg':
        sub = rospy.Subscriber(TOPIC_leg_tracker, PositionMeasurementArray, tracker_cb)
    else:
        print("Argument must be 'people' or 'leg'")
        sys.exit(0)

    rospy.spin()
