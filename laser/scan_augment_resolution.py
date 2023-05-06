import rospy, time

from sensor_msgs.msg import LaserScan

TOPIC_scan_nomap = '/scan_nomap'

class AugmentedResolutionScan(object):

    def __init__(self):
        self.scan_pub = rospy.Publisher("/scan_nomap_augmented", LaserScan, queue_size=1)

    def ScanPub(self):
        rospy.init_node("scan_nomap_augmented_resolution", disable_signals=True)
        rospy.sleep(1)

        scan_sub = rospy.Subscriber("/scan_nomap", LaserScan, self.ScanCB)

        rospy.spin()

    def ScanCB(self, msg):
        laser_scan = LaserScan()
        laser_scan.header = msg.header
        laser_scan.angle_min = msg.angle_min
        laser_scan.angle_max = msg.angle_max
        laser_scan.angle_increment = msg.angle_increment / 2.0
        laser_scan.time_increment = msg.time_increment
        laser_scan.scan_time = msg.scan_time
        laser_scan.range_min = msg.range_min
        laser_scan.range_max = msg.range_max
        laser_scan.ranges = []
        for i in range(1, len(msg.ranges)):
            laser_scan.ranges.append(msg.ranges[i-1])
            new = (msg.ranges[i-1] + msg.ranges[i]) / 2.0
            laser_scan.ranges.append(new)

        laser_scan.intensities = []
        for r in laser_scan.ranges:
            if r < 10.0:
                laser_scan.intensities.append(1.0)
            else:
                laser_scan.intensities.append(0.0)

        self.scan_pub.publish(laser_scan)


# main
if __name__ == "__main__":

    scan = AugmentedResolutionScan()
    scan.ScanPub()