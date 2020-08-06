#!/usr/bin/env python

PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)

import rospy
import sys

from std_msgs.msg import Int16

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy


a = numpy.array([[1.9, 2.1] , [3.2, 4.3]], dtype=numpy.float32)
print"Our Matrix A:\n", (a)
rowA, colA = a.shape
print"\nMatrix A row: ", (rowA)
print"\nMatrix A column: ", (colA)

## Reason tho reshape, rospy.numpy_msg can only publish 1 dimensional array,
##      so convert our matrix A to a 1 dimensional array
newShapeA = rowA * colA
print"\nThis is value to reshape sendMatrix for a: ", (newShapeA)
sendA = a.reshape(newShapeA)
print"\nsendMatrix A:\n", (sendA)
#-------------------------------------------------------------------------------------
print"---------------------------------------------------"
b = numpy.array([[5.9, 6.7] , [7.8, 8.9]], dtype=numpy.float32)
print"\nOur Matrix B:\n", (b)
rowB, colB = b.shape
print"\nMatrix B row: ", (rowB)
print"\nMatrix B column: ", (colB)
## Reason tho reshape, rospy.numpy_msg can only publish 1 dimensional array,
##      so convert our matrix A to a 1 dimensional array
newShapeB = rowB * colB
print"\nThis is value to reshape sendMatrix for b: ", (newShapeB)
sendB = b.reshape(newShapeB)
print"\nsendMatrix B:\n", (sendB)
#------------------------------------------------------------------------------------
class myMatrixA:
    def __init__(self):
        self.matrix = 0
        self.matrixRow = 0
        self.matrixCol = 0
        self.subMatrix = rospy.Subscriber("floatsAToMaster", numpy_msg(Floats), self.callback)
        self.subRow = rospy.Subscriber("rowAToMaster", Int16, self.grabRow)
        self.subCol = rospy.Subscriber("colAToMaster", Int16, self.grabCol)

    def callback(self, data):
        self.matrix = data
        print "Matrix from worker A \n"
        print(self.matrix)

    def grabRow(self, data):
        self.matrixRow = data
        print "Matrix row from worker A\n"
        print(self.matrixRow)

    def grabCol(self, data):
        self.matrixCol = data
        print "Matrix col from worker A\n"
        print(self.matrixCol)
   
def initialize():
    rospy.init_node('master', anonymous=True)

def talker():
    pubMatrixA = rospy.Publisher('floatsAToWorker', numpy_msg(Floats), queue_size=10, latch = True)
    pubMatrixARow = rospy.Publisher('rowAToWorker', Int16, queue_size=9, latch = True)
    pubMatrixACol = rospy.Publisher('colAToWorker', Int16, queue_size=8, latch = True)

    pubMatrixB = rospy.Publisher('floatsBToWorker', numpy_msg(Floats), queue_size=10, latch = True)
    pubMatrixBRow = rospy.Publisher('rowBToWorker', Int16, queue_size=9, latch = True)
    pubMatrixBCol = rospy.Publisher('colBToWorker', Int16, queue_size=8, latch = True)

    ##rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    connectionsReady = False

    while not connectionsReady:
        connectionMatrixA = pubMatrixA.get_num_connections()
        connectionMatrixARow = pubMatrixARow.get_num_connections()
        connectionMatrixACol = pubMatrixACol.get_num_connections()
        if connectionMatrixA > 0 and connectionMatrixARow > 0 and connectionMatrixACol > 0:
            pubMatrixA.publish(sendA)
            pubMatrixARow.publish(rowA)
            pubMatrixACol.publish(colA)
            print "Messages to Worker A published. \n"
            connectionsReady = True
        else:
            rate.sleep()



   
    ##pubMatrixA.publish(sendA)
    ##pubMatrixARow.publish(rowA)
    ##pubMatrixACol.publish(colA)

    ##print "Messages to Worker A published. \n"

    ##pubMatrixB.publish(sendB)
    ##pubMatrixBRow.publish(rowB)
    ##pubMatrixBCol.publish(colB)

def listenerA(args):
    obc = myMatrixA()
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
    talker()
    listenerA(sys.argv)
