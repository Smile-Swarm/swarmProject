#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def initialize():
    rospy.init_node('worker', anonymous=True)

def talker(data):
    pub = rospy.Publisher('s_to_m', Int16, queue_size=10) 
    #rate = rospy.Rate(1) #1hz
    #while not rospy.is_shutdown():
    number = data.data + 1
    msg = "Data is " + str(number)
    rospy.loginfo(msg)
    pub.publish(number)
        #rate.sleep()

#def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + 'Data is now %s', data.data)

def listener():
    rospy.Subscriber('m_to_s', Int16, talker)
    rospy.spin()


if __name__ == '__main__':
    try:
        initialize()
        listener()
        #talker()
        #listener()

    except rospy.ROSInterruptException:
        pass
