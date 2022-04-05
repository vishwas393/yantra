#!/usr/bin/env python

import math
#import rospy
import sys
import numpy as np
#from yantra.msg import *
#from yantra.srv import *

from predefined_matrices import *

K = 0.00001*(np.eye(3))

def callback_fn(P, Q_init):
    err = 10
    cnt = 0
    q = Q_init
    while (err != 0):


        J = jacobian(q)
        EEPOS = end_effector_position(q)

        tmp = np.transpose(J)@(K@(P - EEPOS))
        qn = q + tmp
        err = np.sum((abs(qn-q) > 0.01))
        q = qn
        cnt += 1
        print(err, end="")

    print("Iterations: " + str(cnt))
    qn = np.array(qn)
    return qn

        
if __name__ == "__main__":
    P = np.array([300,300,300]).reshape(3,1)
    Q_init = np.array([math.pi/4, 0, 0, 0, 0]).reshape(5,1)
    val = callback_fn(P, Q_init)
    print(val)
