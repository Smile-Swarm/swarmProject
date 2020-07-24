#!/usr/bin/env python

PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy
def talker():
    pubA = rospy.Publisher('floatsA', numpy_msg(Floats), queue_size=10)
    pubB = rospy.Publisher('floatsB', numpy_msg(Floats), queue_size=10)

    pub = rospy.Publisher('floatsB', numpy_msg(Floats), queue_size=10)
    #pub = rospy.Publisher('floats', numpy_msg(Floats))
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        #a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
        a = numpy.array([1.1, 2.2], dtype=numpy.float32)
        b = numpy.array([3.3, 4.4], dtype=numpy.float32)
        pubA.publish(a)
        pubB.publish(b)
        r.sleep()

if __name__ == '__main__':
    talker()
