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
    
    JE[0,0] = - 56.0*sin(d1) + 56.0*sin(d5)*(- 1.0*cos(d3)*(sin(d1)*sin(d2)) - 1.0*sin(d3)*(cos(d2)*sin(d1))) - 56.0*cos(d5)*(cos(d4)*(cos(d3)*(cos(d2)*sin(d1)) - 1.0*sin(d3)*(sin(d1)*sin(d2))) - 1.0*sin(d4)*(cos(d1))) - 170.0*cos(d3)*(sin(d1)*sin(d2)) - 56.0*cos(d3)*(cos(d2)*sin(d1)) + 56.0*sin(d3)*(sin(d1)*sin(d2)) - 170.0*sin(d3)*(cos(d2)*sin(d1)) - 200.0*cos(d2)*sin(d1)

    JE[1,0] = 56.0*cos(d1) - 56.0*cos(d3)*(- 1.0*cos(d1)*cos(d2)) + 56.0*sin(d5)*(- 1.0*sin(d3)*( - 1.0*cos(d1)*cos(d2)) + cos(d3)*(cos(d1)*sin(d2))) - 56.0*cos(d5)*(cos(d4)*(cos(d3)*(- 1.0*cos(d1)*cos(d2)) + sin(d3)*(cos(d1)*sin(d2))) - 1.0*sin(d4)*(sin(d1))) - 170.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) + 170.0*cos(d3)*(cos(d1)*sin(d2)) - 56.0*sin(d3)*(cos(d1)*sin(d2) ) + 200.0*cos(d1)*cos(d2)

    JE[2,0] = 0

    JE[0,1] = 56.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) - 170.0*cos(d3)*(- 1.0*cos(d1)*cos(d2)) - 56.0*sin(d5)*(1.0*cos(d3)*(- 1.0*cos(d1)*cos(d2)) + 1.0*sin(d3)*(cos(d1)*sin(d2))) + 56.0*cos(d5)*(cos(d4)*(1.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) - cos(d3)*(cos(d1)*sin(d2) ))) - 56.0*cos(d3)*(cos(d1)*sin(d2)) - 170.0*sin(d3)*(cos(d1)*sin(d2)) - 200.0*cos(d1)*sin(d2)

    JE[1,1] = 170.0*cos(d3)*(cos(d2)*sin(d1)) - 56.0*cos(d5)*(cos(d4)*(cos(d3)*(sin(d1)*sin(d2)) + sin(d3)*(cos(d2)*sin(d1)))) - 56.0*cos(d3)*(sin(d1)*sin(d2)) - 200.0*sin(d1)*sin(d2) - 170.0*sin(d3)*(sin(d1)*sin(d2) ) - 56.0*sin(d3)*(cos(d2)*sin(d1)) + 56.0*sin(d5)*(cos(d3)*(cos(d2)*sin(d1)) - 1.0*sin(d3)*(sin(d1)*sin(d2)))

    JE[2,1] = 200.0*cos(d2) + 56.0*sin(d5)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)) - 56.0*sin(d2)*sin(d3) - 56.0*cos(d5)*(cos(d4)*(sin(d2)*sin(d3) - 1.0*cos(d2)*cos(d3))) + 56.0*cos(d2)*cos(d3) + 170.0*cos(d2)*sin(d3) + 170.0*cos(d3)*sin(d2)

    JE[0,2] = 56.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) - 170.0*cos(d3)*(- 1.0*cos(d1)*cos(d2)) - 56.0*sin(d5)*(1.0*cos(d3)*(- 1.0*cos(d1)*cos(d2)) + 1.0*sin(d3)*(cos(d1)*sin(d2))) + 56.0*cos(d5)*(cos(d4)*(1.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) - cos(d3)*(cos(d1)*sin(d2)))) - 56.0*cos(d3)*(cos(d1)*sin(d2)) - 170.0*sin(d3)*(cos(d1)*sin(d2))

    JE[1,2] = 170.0*cos(d3)*(cos(d2)*sin(d1)) - 56.0*cos(d3)*(sin(d1)*sin(d2)) - 56.0*cos(d5)*(cos(d4)*(cos(d3)*(sin(d1)*sin(d2)) + sin(d3)*(cos(d2)*sin(d1)))) - 170.0*sin(d3)*(sin(d1)*sin(d2)) - 56.0*sin(d3)*(cos(d2)*sin(d1)) + 56.0*sin(d5)*(cos(d3)*(cos(d2)*sin(d1)) - 1.0*sin(d3)*(sin(d1)*sin(d2)))

    JE[2,2] = 56.0*sin(d5)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)) - 56.0*sin(d2)*sin(d3) - 56.0*cos(d5)*(cos(d4)*(sin(d2)*sin(d3) - 1.0*cos(d2)*cos(d3))) + 56.0*cos(d2)*cos(d3) + 170.0*cos(d2)*sin(d3) + 170.0*cos(d3)*sin(d2)

    JE[0,3] = 56.0*cos(d5)*(sin(d4)*(cos(d3)*(- 1.0*cos(d1)*cos(d2)) + sin(d3)*(cos(d1)*sin(d2))) + cos(d4)*(sin(d1)))

    JE[1,3] = - 56.0*cos(d5)*(sin(d4)*(cos(d3)*(cos(d2)*sin(d1)) - 1.0*sin(d3)*(sin(d1)*sin(d2))) + cos(d4)*(cos(d1)))

    JE[2,3] = 56.0*cos(d5)*(- 1.0*sin(d4)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)))

    JE[0,4] = 56.0*sin(d5)*(cos(d4)*(cos(d3)*(- 1.0*cos(d1)*cos(d2)) + sin(d3)*(cos(d1)*sin(d2))) - 1.0*sin(d4)*(sin(d1))) + 56.0*cos(d5)*(- 1.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) + cos(d3)*(cos(d1)*sin(d2)))

    JE[1,4] = - 56.0*cos(d5)*(- 1.0*cos(d3)*(sin(d1)*sin(d2)) - 1.0*sin(d3)*(cos(d2)*sin(d1))) - 56.0*sin(d5)*(cos(d4)*(cos(d3)*(cos(d2)*sin(d1)) - 1.0*sin(d3)*(sin(d1)*sin(d2))) - 1.0*sin(d4)*(cos(d1)))

    JE[2,4] = 56.0*cos(d5)*(sin(d2)*sin(d3) - 1.0*cos(d2)*cos(d3)) - 56.0*sin(d5)*(cos(d4)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)))


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
    d6 = d[5]

    t1 = 56*cos(d1) + 70*sin(d4)*(cos(d1)*cos(d2)*cos(d3) - cos(d1)*sin(d2)*sin(d3)) + 56*sin(d5)*(cos(d1)*cos(d2)*sin(d3) + cos(d1)*cos(d3)*sin(d2)) + 56*cos(d5)*(sin(d1)*sin(d4) + cos(d4)*(cos(d1)*cos(d2)*cos(d3) - cos(d1)*sin(d2)*sin(d3))) + 205*cos(d1)*cos(d2) - 70*cos(d4)*sin(d1) + 56*cos(d1)*cos(d2)*cos(d3) + 167*cos(d1)*cos(d2)*sin(d3) + 167*cos(d1)*cos(d3)*sin(d2) - 56*cos(d1)*sin(d2)*sin(d3)

    t2 = 56*sin(d1) + 70*sin(d4)*(cos(d2)*cos(d3)*sin(d1) - sin(d1)*sin(d2)*sin(d3)) + 56*sin(d5)*(cos(d2)*sin(d1)*sin(d3) + cos(d3)*sin(d1)*sin(d2)) + 56*cos(d5)*(cos(d4)*(cos(d2)*cos(d3)*sin(d1) - sin(d1)*sin(d2)*sin(d3)) - cos(d1)*sin(d4)) + 70*cos(d1)*cos(d4) + 205*cos(d2)*sin(d1) + 56*cos(d2)*cos(d3)*sin(d1) + 167*cos(d2)*sin(d1)*sin(d3) + 167*cos(d3)*sin(d1)*sin(d2) - 56*sin(d1)*sin(d2)*sin(d3)
    
    t3 = 205*sin(d2) + 167*sin(d2)*sin(d3) + 70*sin(d4)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)) + 56*sin(d5)*(sin(d2)*sin(d3) - cos(d2)*cos(d3)) - 167*cos(d2)*cos(d3) + 56*cos(d2)*sin(d3) + 56*cos(d3)*sin(d2) + 56*cos(d4)*cos(d5)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)) + 283


    return np.array([t1, t2, t3]).reshape(3,1)


def main():
	dd = [0, 0, 0, 0, 0, 0]
	p = end_effector_position(dd)
	print(p)
	
main()
