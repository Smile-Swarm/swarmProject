#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

number = 1

def initialize():
    rospy.init_node('master', anonymous=True)
    #global number
    #number = 1

def talker():
    global number 
    pub = rospy.Publisher('m_to_s', Int16, queue_size=10) 
    rate = rospy.Rate(1) #1hz
    while not rospy.is_shutdown():
        msg = "Data is " + str(number)
        rospy.loginfo(msg)
        pub.publish(number)
        listener()
        rate.sleep()

def callback(data):
    global number
    number = data
    rospy.loginfo(rospy.get_caller_id() + 'Data is now %s', data.data)

def listener():
    rospy.Subscriber('s_to_m', Int16, callback)
    #rospy.spin()


if __name__ == '__main__':
    try:
        initialize()
        talker()
        #listener()

    except rospy.ROSInterruptException:
        pass
