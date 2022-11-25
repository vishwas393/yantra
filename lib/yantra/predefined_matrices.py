#!/usr/bin/env python

from  math import *
import numpy as np

'''
# Brief: Predefined Analytical Jacobian vector.
         Retrieved from del(end-effector position)/dq
# Input : Numpy vector of joint angles in radians
# Output: Jacobian vector  

'''
def jacobian(d): 
    #print(type(d))
    d1 = d[0]
    d2 = d[1]
    d3 = d[2]
    d4 = d[3]
    d5 = d[4]

    #print(str(type(d1)) + "  " + str(d1))
    #print(str(type(d4)) + "  " + str(d4))

    JE = np.zeros((3, 5))
    
    JE[0,0] = 56*cos(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4)) - 104*cos(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 56*sin(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 205*cos(d2 + pi/2)*sin(d1) - 104*sin(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4)) - 291*cos(d3)*cos(d2 + pi/2)*sin(d1) + 291*sin(d1)*sin(d3)*sin(d2 + pi/2)

    JE[1,0] = 104*cos(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) + 56*sin(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) + 56*cos(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3))) - 104*sin(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3))) + 205*cos(d1)*cos(d2 + pi/2) + 291*cos(d1)*cos(d3)*cos(d2 + pi/2) - 291*cos(d1)*sin(d3)*sin(d2 + pi/2)

    JE[2,0] = 0

    JE[0,1] = 104*cos(d4)*sin(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) - 104*cos(d5)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3)) - 56*sin(d5)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3)) - 205*cos(d1)*sin(d2 + pi/2) - 291*cos(d1)*cos(d3)*sin(d2 + pi/2) - 291*cos(d1)*cos(d2 + pi/2)*sin(d3) - 56*cos(d4)*cos(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2))

    JE[1,1] = 104*cos(d4)*sin(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 104*cos(d5)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) - 56*sin(d5)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) - 56*cos(d4)*cos(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 205*sin(d1)*sin(d2 + pi/2) - 291*cos(d3)*sin(d1)*sin(d2 + pi/2) - 291*cos(d2 + pi/2)*sin(d1)*sin(d3)

    JE[2,1] = 205*cos(d2 + pi/2) - 291*sin(d3)*sin(d2 + pi/2) - 104*cos(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) - 56*sin(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) + 291*cos(d3)*cos(d2 + pi/2) - 56*cos(d4)*cos(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) + 104*cos(d4)*sin(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3))

    JE[0,2] = 104*cos(d4)*sin(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) - 56*sin(d5)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3)) - 104*cos(d5)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3)) - 291*cos(d1)*cos(d3)*sin(d2 + pi/2) - 291*cos(d1)*cos(d2 + pi/2)*sin(d3) - 56*cos(d4)*cos(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2))

    JE[1,2] = 104*cos(d4)*sin(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 56*sin(d5)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) - 56*cos(d4)*cos(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 104*cos(d5)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) - 291*cos(d3)*sin(d1)*sin(d2 + pi/2) - 291*cos(d2 + pi/2)*sin(d1)*sin(d3)

    JE[2,2] = 291*cos(d3)*cos(d2 + pi/2) - 104*cos(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) - 56*sin(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) - 291*sin(d3)*sin(d2 + pi/2) - 56*cos(d4)*cos(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) + 104*cos(d4)*sin(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3))

    JE[0,3] = 56*cos(d5)*(sin(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3)) + cos(d4)*sin(d1)) - 104*sin(d5)*(sin(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3)) + cos(d4)*sin(d1))

    JE[1,3] = 56*cos(d5)*(sin(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) - cos(d1)*cos(d4)) - 104*sin(d5)*(sin(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) - cos(d1)*cos(d4))

    JE[2,3] = 56*cos(d5)*sin(d4)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) - 104*sin(d4)*sin(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2))

    JE[0,4] = 56*cos(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) - 104*sin(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) - 104*cos(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3))) - 56*sin(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3)))

    JE[1,4] = 56*cos(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 104*sin(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) + 104*cos(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4)) + 56*sin(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4))

    JE[2,4] = 56*cos(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) - 104*sin(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) + 104*cos(d4)*cos(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) + 56*cos(d4)*sin(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2))


    return JE

'''
# Brief: Predefined end effector position vector.
         Retrieved from Transformation matrix T_0_5.
# Input : Numpy vector of joint angles in radians
# Output: Position vector of End Effector  

'''
def end_effector_position(d):
    d1 = d[0]
    d2 = d[1]
    d3 = d[2]
    d4 = d[3]
    d5 = d[4]
    #d6 = d[5]

    t1 = 104*cos(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) + 56*sin(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) + 56*cos(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3))) - 104*sin(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3))) + 205*cos(d1)*cos(d2 + pi/2) + 291*cos(d1)*cos(d3)*cos(d2 + pi/2) - 291*cos(d1)*sin(d3)*sin(d2 + pi/2)

    t2 = 205*cos(d2 + pi/2)*sin(d1) + 104*cos(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) + 56*sin(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 56*cos(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4)) + 104*sin(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4)) + 291*cos(d3)*cos(d2 + pi/2)*sin(d1) - 291*sin(d1)*sin(d3)*sin(d2 + pi/2)
    
    t3 = 205*sin(d2 + pi/2) + 291*cos(d3)*sin(d2 + pi/2) + 291*cos(d2 + pi/2)*sin(d3) + 104*cos(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) + 56*sin(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) - 56*cos(d4)*cos(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) + 104*cos(d4)*sin(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) + 283


    return np.array([t1, t2, t3]).reshape(3,1)
