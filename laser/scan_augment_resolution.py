import rospy, time

from sensor_msgs.msg import LaserScan

TOPIC_scan_nomap = '/scan_nomap'

class AugmentedResolutionScan(object):

    def __init__(self):
        self.scan_pub = rospy.Publisher("/scan_nomap_augmented", LaserScan, queue_size=1)
        self.max_scan_range = 10.0

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
            r0 = msg.ranges[i-1]
            r1 = msg.ranges[i]
            laser_scan.ranges.append(r0)
            if r1 <= self.max_scan_range and abs(r1-r0) < 0.4:
                new = (r0 + r1) / 2.0
            else:
                new = msg.range_max+1
            laser_scan.ranges.append(new)

        laser_scan.intensities = []
        for r in laser_scan.ranges:
            if r <= self.max_scan_range:
                laser_scan.intensities.append(1.0)
            else:
                laser_scan.intensities.append(0.0)

        self.scan_pub.publish(laser_scan)


# main
if __name__ == "__main__":

    print("Running...")
    scan = AugmentedResolutionScan()
    scan.ScanPub()
