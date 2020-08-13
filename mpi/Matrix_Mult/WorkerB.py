#!/usr/bin/env python
PKG = 'numpy_tutorial'
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
    global matrixRow
    matrixRow = data
    #print("row", matrixRow)

def grabCol(data):
    global matrixCol
    matrixCol = data
    #print("col", matrixCol)

rospy.init_node('workerb')
subMatrix = rospy.Subscriber("floatsB", numpy_msg(Floats), callback)
subVec = rospy.Subscriber("matrixA_Row1", numpy_msg(Floats), grabVect) ## Version 3: added subscriber
subRow = rospy.Subscriber("rowB", Int16, grabRow)
subCol = rospy.Subscriber("colB", Int16, grabCol)
pub = rospy.Publisher("resultB", numpy_msg(Floats), queue_size=1, latch = True)


rate = rospy.Rate(1)
ctrl_c = False
#if in shutdown, completely stop movement
def shutdownhook():
    global ctrl_c
    print "shutdown time! Stop the robot"

    ctrl_c = True

rospy.on_shutdown(shutdownhook)

while not ctrl_c:
    try:
        ## Version 3: added matrix mult. and variables to print
        rospy.sleep(5)
        print("\n\n\n\n\n\n\n\n")
        print"Received matrix:\n", (matrix)
        newMatrix = matrix.data.reshape([matrixRow.data, matrixCol.data])
        myVect = vect.data
        result = myVect.dot(newMatrix)
        print"\nDecoded Matrix: \n", (newMatrix), "\n\nVector(Row of Matrix A):\n", (myVect)
        print"\n\n Vector dot product matrix", myVect.dot(newMatrix)
        pub.publish(result)
    except KeyboardInterrupt:
        print("Shutting down")
    rate.sleep()
