#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan

def callback(msg) :
   # print len(msg.ranges)
    print 'value at 0 degrees'
    print msg.ranges[0]
    print 'value at 90 degrees'
    print msg.ranges[360]
    print 'value at 180 degrees'
    print msg.ranges[719]


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback) # creates subscriber to /scan topic
rospy.spin()
