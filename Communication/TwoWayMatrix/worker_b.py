#!/usr/bin/env python
PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)
import sys

import rospy
import numpy
from std_msgs.msg import Int16
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def talker(self):
        pubMatrixB = rospy.Publisher('floatsBToMaster', numpy_msg(Floats), queue_size=10, latch = True)
        pubMatrixBRow = rospy.Publisher('rowBToMaster', Int16, queue_size=9, latch = True)
        pubMatrixBCol = rospy.Publisher('colBToMaster', Int16, queue_size=8, latch = True)

        print "Talker reached \n"

        rate = rospy.Rate(1) # 10hz
        connectionsReady = False

        while not connectionsReady:
            connectionMatrixB = pubMatrixB.get_num_connections()
            connectionMatrixBRow = pubMatrixBRow.get_num_connections()
            connectionMatrixBCol = pubMatrixBCol.get_num_connections()
            if connectionMatrixB > 0 and connectionMatrixBRow > 0 and connectionMatrixBCol > 0:
                pubMatrixB.publish(self.matrix.data)
                pubMatrixBRow.publish(self.matrixRow.data)
                pubMatrixBCol.publish(self.matrixCol.data)
                print "Messages to Master published. \n"
                connectionsReady = True
            else:
                rate.sleep()

class myMatrix:
    def __init__(self):
        self.matrix = 0
        self.matrixRow = 0
        self.matrixCol = 0
        self.subMatrix = rospy.Subscriber("floatsBToWorker", numpy_msg(Floats), self.callback)
        self.subRow = rospy.Subscriber("rowBToWorker", Int16, self.grabRow)
        self.subCol = rospy.Subscriber("colBToWorker", Int16, self.grabCol)

    def callback(self, data):
        self.matrix = data
        print(self.matrix)
        talker(self)

    def grabRow(self, data):
        self.matrixRow = data
        print(self.matrixRow)

    def grabCol(self, data):
        self.matrixCol = data
        print(self.matrixCol)

def initialize():
    rospy.init_node('worker_b', anonymous=True)

def listener(args):
    obc = myMatrix()
    try:
        rospy.spin()
        #printSomething()
        print("\n\n\n\n\n\n\n\n")
        print"Received matrix:\n", (obc.matrix)
        #print(obc.matrixRow)
        #print(obc.matrixCol)
        #reshapeVal = obc.matrixRow.data * obc.matrixCol.data
        #print(reshapeVal)
        newMatrix = obc.matrix.data.reshape([obc.matrixRow.data, obc.matrixCol.data])
        print"\nDecoded Matrix: \n", (newMatrix)
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    initialize()
    listener(sys.argv)
