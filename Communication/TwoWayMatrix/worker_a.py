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
        pubMatrixA = rospy.Publisher('floatsAToMaster', numpy_msg(Floats), queue_size=10, latch = True)
        pubMatrixARow = rospy.Publisher('rowAToMaster', Int16, queue_size=9, latch = True)
        pubMatrixACol = rospy.Publisher('colAToMaster', Int16, queue_size=8, latch = True)

        print "Talker reached \n"

        rate = rospy.Rate(1) # 10hz
        connectionsReady = False

        while not connectionsReady:
            connectionMatrixA = pubMatrixA.get_num_connections()
            connectionMatrixARow = pubMatrixARow.get_num_connections()
            connectionMatrixACol = pubMatrixACol.get_num_connections()
            if connectionMatrixA > 0 and connectionMatrixARow > 0 and connectionMatrixACol > 0:
                pubMatrixA.publish(self.matrix.data)
                pubMatrixARow.publish(self.matrixRow.data)
                pubMatrixACol.publish(self.matrixCol.data)
                print "Messages to Master published. \n"
                connectionsReady = True
            else:
                rate.sleep()

class myMatrix:
    def __init__(self):
        self.matrix = 0
        self.matrixRow = 0
        self.matrixCol = 0
        self.subMatrix = rospy.Subscriber("floatsAToWorker", numpy_msg(Floats), self.callback)
        self.subRow = rospy.Subscriber("rowAToWorker", Int16, self.grabRow)
        self.subCol = rospy.Subscriber("colAToWorker", Int16, self.grabCol)

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
    rospy.init_node('worker_a', anonymous=True)
   
    
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

        ##talker(obc)

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    initialize()
    listener(sys.argv)
