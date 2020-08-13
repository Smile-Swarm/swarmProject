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
print"Matrix A:\n", (a)
rowA, colA = a.shape

#Reason tho reshape, rospy.numpy_msg can only publish 1 dimensional array, so convert our matrix A to a 1 dimensional array
newShapeA = rowA * colA
sendA = a.reshape(newShapeA)

matrixA_row0 = a[0]
matrixA_row1 = a[1]

b = numpy.array([[5.9, 6.7] , [7.8, 8.9]], dtype=numpy.float32)
print"\nMatrix B:\n", (b)
rowB, colB = b.shape
#Reason tho reshape, rospy.numpy_msg can only publish 1 dimensional array, so convert our matrix A to a 1 dimensional array
newShapeB = rowB * colB
sendB = b.reshape(newShapeB)

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
pubMatrixA_Row0 = rospy.Publisher('matrixA_Row0', numpy_msg(Floats), queue_size=10, latch = True)

pubMatrixB = rospy.Publisher('floatsB', numpy_msg(Floats), queue_size=10, latch = True)
pubMatrixBRow = rospy.Publisher('rowB', Int16, queue_size=9, latch = True)
pubMatrixBCol = rospy.Publisher('colB', Int16, queue_size=8, latch = True)
pubMatrixA_Row1 = rospy.Publisher('matrixA_Row1', numpy_msg(Floats), queue_size=10, latch = True)

sub1 = rospy.Subscriber("resultA", numpy_msg(Floats), callbackA)
sub2 = rospy.Subscriber("resultB", numpy_msg(Floats), callbackB)

rate = rospy.Rate(1)
ctrl_c = False
#if in shutdown, completely stop movement
def shutdownhook():
    global ctrl_c
    print "shutdown time!"
    ctrl_c = True

rospy.on_shutdown(shutdownhook)
final = numpy.empty((2,2), dtype=numpy.float32)
i = 0
while not ctrl_c:
    pubMatrixA.publish(sendA)
    pubMatrixARow.publish(rowA)
    pubMatrixACol.publish(colA)
    pubMatrixA_Row0.publish(matrixA_row0)

    pubMatrixB.publish(sendB)
    pubMatrixBRow.publish(rowB)
    pubMatrixBCol.publish(colB)
    pubMatrixA_Row1.publish(matrixA_row1)

    i = i + 1
    if i == 8:
        final[0] = resultA.data
        final[1] = resultB.data
        print("\nMultiplication Result:")
        print(final)
    rate.sleep()
