#!/usr/bin/env python
import sys
import rospy
import numpy
from std_msgs.msg import Int16
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

matrix = 0
vect = 0
matrixRow = 0
matrixCol = 0

def callback(data):
    global matrix
    matrix = data
    #print(matrix)

## Version 3: new function added
def grabVect(data):
    global vect
    vect = data
    print(vect)

def grabRow(data):
    global matrixRox
    matrixRow = data
    #print(matrixRow)

def grabCol(data):
    global matrixCol
    matrixCol = data
    #print(self.matrixCol)

rospy.init_node('workera')
subMatrix = rospy.Subscriber("floatsB", numpy_msg(Floats), callback)
subVec = rospy.Subscriber("matrixA_Row0", numpy_msg(Floats), grabVect) ## Version 3: added subscriber
subRow = rospy.Subscriber("rowB", Int16, grabRow)
subCol = rospy.Subscriber("colB", Int16, grabCol)
pub = rospy.Publisher('result', numpy_msg(Floats), queue_size=1, latch = True)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    try:
        ## Version 3: added matrix mult. and variables to print
        rospy.spin()
        print("\n\n\n\n\n\n\n\n")
        print"Received matrix:\n", (matrix)
        print("row", matrixCol)
        print("col", matrixRow)
        newMatrix = numpy.reshape(matrix, (matrixRow, matrixCol))
        myVect = vect
        result = myVect.dot(newMatrix)
        print"\nDecoded Matrix: \n", (newMatrix), "\n\nVector(Row of Matrix A):\n", (myVect)
        print"\n\n Vector dot product matrix", myVect.dot(newMatrix)
        pub.Publish(result)
    except KeyboardInterrupt:
        print("Shutting down")
    rate.sleep()

    
