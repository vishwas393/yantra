#!/usr/bin/env python

import math
import rospy
import numpy as np
from yantra.msg import *
from yantra.srv import *
from yantra.predefined_matrices import *



def callback_fn(req):
    err = 10
    cnt = 0
    qt = req.Q_init
    q = np.array(qt.j_values).reshape(5,1)
    K = 0.000001*(np.eye(3))
    while (err != 0):

        J = jacobian(q)
        EEPOS = end_effector_position(q)


        #print(q.shape)
        #print(np.transpose(J).shape)
        #print(K.shape)
        #print(EEPOS.shape)
        #print(np.array(req.P.pos).reshape(3,1)) 

        qn = q + (np.transpose(J)@K)@(np.array(req.P.pos).reshape(3,1) - EEPOS)
        #print(qn)
        err = np.sum((abs(qn-q) > 0.01))
        q = qn
        #print(q)
        cnt += 1
        print(cnt)
    
    #print("out of the loop")
    ret = JointValues()
    ret.j_values = qn
    #print("About to exit")
    return InverseKinematicsResponse(ret)

        

def inverse_kinematics_server():
    rospy.init_node('IK_server')
    srvc = rospy.Service('inverse_kinematics_server', InverseKinematics, callback_fn)
    print("Inverse_Kimatics_Server Online!")
    rospy.spin()


if __name__ == "__main__":
    inverse_kinematics_server()
