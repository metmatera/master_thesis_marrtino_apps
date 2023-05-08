import rosbag
import sys

def getTime(secs, nsecs):
    return secs + nsecs*pow(10,-9)

if __name__ == '__main__':

    bagfile = sys.argv[1]
    txtfile = sys.argv[2]

    bag = rosbag.Bag(bagfile)

    f = open(txtfile, 'w')
    f.write('# /odom extrapolation of ' + bagfile + '\n')

    init_flag = False
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        if init_flag == False:
            t0 = getTime(t.secs,t.nsecs)
            x0 = msg.pose.pose.position.x
            y0 = msg.pose.pose.position.y
            init_flag = True

        time = getTime(t.secs,t.nsecs)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        f.write(str(time-t0) + ',' + str(x-x0) + ',' + str(y-y0) + '\n')

    f.close()
    bag.close()
