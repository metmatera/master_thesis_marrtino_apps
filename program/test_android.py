#!/usr/bin/env python

from robot_cmd_ros import *

begin()

while True:
   print '\033c'
   print 'Imu',          accel_gyro() 
   print 'Illuminance',  illuminance() 
   print 'NavSat',       sat_nav() 
   print 'Magnetometer', magnetometer() 
   wait(0)

end()



