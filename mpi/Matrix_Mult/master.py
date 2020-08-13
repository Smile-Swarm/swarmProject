#!/usr/bin/env python
import tf
import time
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
print(type(a))

## //Version 3 added
matrixA_row0 = a[0]
print"\nPrinting first row of matrix A:\n", matrixA_row0

matrixA_row1 = a[1]
print"\nPrinting second row of matrix A:\n", matrixA_row1
## //Ver3 end

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

resultA = 0
resultB = 0

def callbackA(msg):
    global resultA
    resultA = msg

def callbackB(msg):
    global resultB
    resultB = msg

rospy.init_node('master')
pubMatrixA = rospy.Publisher('floatsA', numpy_msg(Floats), queue_size=10, latch = True)
pubMatrixARow = rospy.Publisher('rowA', Int16, queue_size=9, latch = True)
pubMatrixACol = rospy.Publisher('colA', Int16, queue_size=8, latch = True)
## //Version 3 added
pubMatrixA_Row0 = rospy.Publisher('matrixA_Row0', numpy_msg(Floats), queue_size=10, latch = True)
## //Ver3 end

pubMatrixB = rospy.Publisher('floatsB', numpy_msg(Floats), queue_size=10, latch = True)
pubMatrixBRow = rospy.Publisher('rowB', Int16, queue_size=9, latch = True)
pubMatrixBCol = rospy.Publisher('colB', Int16, queue_size=8, latch = True)
## //Version 3 added
pubMatrixA_Row1 = rospy.Publisher('matrixA_Row1', numpy_msg(Floats), queue_size=10, latch = True)
## //Ver3 end
sub1 = rospy.Subscriber("resultA", numpy_msg(Floats), callbackA)
sub2 = rospy.Subscriber("resultB", numpy_msg(Floats), callbackB)

rate = rospy.Rate(1)
ctrl_c = False
#if in shutdown, completely stop movement
def shutdownhook():
    global ctrl_c
    print "shutdown time! Stop the robot"

    ctrl_c = True

rospy.on_shutdown(shutdownhook)
final = numpy.empty((2,2), dtype=numpy.float32)
i = 0
while not ctrl_c:
    pubMatrixA.publish(sendA)
    pubMatrixARow.publish(rowA)
    pubMatrixACol.publish(colA)
    ## //Version 3 added
    pubMatrixA_Row0.publish(matrixA_row0)
    ## //Ver3 end

    pubMatrixB.publish(sendB)
    pubMatrixBRow.publish(rowB)
    pubMatrixBCol.publish(colB)
    ## //Version 3 added
    pubMatrixA_Row1.publish(matrixA_row1)
    print(resultA)
    print(resultB)
    #print(type(resultA))
    i = i + 1
    if i > 5:
        final[0] = resultA.data
        final[1] = resultB.data
        print("Multiplied Matrix:")
        print(final)
    ## //Ver3 end
    rate.sleep()
