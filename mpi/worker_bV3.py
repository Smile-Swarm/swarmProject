#!/usr/bin/env python
PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)
import sys

import rospy
import numpy
from std_msgs.msg import Int16
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

class myMatrix:
    def __init__(self):
        self.matrix = 0
        self.vect = 0
        self.matrixRow = 0
        self.matrixCol = 0
        self.subMatrix = rospy.Subscriber("floatsB", numpy_msg(Floats), self.callback)
        self.subVec = rospy.Subscriber("matrixA_Row1", numpy_msg(Floats), self.grabVect)
        self.subRow = rospy.Subscriber("rowB", Int16, self.grabRow)
        self.subCol = rospy.Subscriber("colB", Int16, self.grabCol)

    def callback(self, data):
        self.matrix = data
        #print(self.matrix)

    def grabVect(self, data):
        self.vect = data
        print(self.vect)

    def grabRow(self, data):
        self.matrixRow = data
        #print(self.matrixRow)

    def grabCol(self, data):
        self.matrixCol = data
        #print(self.matrixCol)

#def printSomething():
    #print"Hello World"

def main(args):
    obc = myMatrix()
    rospy.init_node('listener', anonymous=True)
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
        myVect = obc.vect.data
        print"\nDecoded Matrix: \n", (newMatrix), "\n\nVector(Row of Matrix A):\n", (myVect)
        print"\n\n Vector dot product matrix", myVect.dot(newMatrix)
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
