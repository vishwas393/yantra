#!/usr/bin/env python3

import math
import rospy
import numpy as np
from yantra.msg import *
from yantra.srv import *
from yantra.predefined_matrices import *


def calculate_imposed_velocity(_qn, _time):
    # Calculating Vi,k
    lin_vel = np.zeros((5,3))
    ang_vel = np.zeros((5,4))

    for i in range(lin_vel.shape[0]):
        for k in range(lin_vel.shape[1]):
            lin_vel[i,k] = (_qn[k+1].j_value[i] - _qn[k].j_value[i]) / (_time[k+1] - _time[k])
            print("lin_vel ", i, k, " : ", lin_vel[i,k], " , ")

    for i in range(ang_vel.shape[0]):
        for k in range(1, ang_vel.shape[1]-1):
            ang_vel[i,k] = 0.5*(lin_vel[i,k-1]+lin_vel[i, k]) if (np.sign(lin_vel[i,k-1]) == np.sign(lin_vel[i, k])) else 0
            print("ang_vel ", i, k, " : " ,ang_vel[i,k], " , ")

    return ang_vel


def callback_fn(req):
    Qn = req.Q
    Qn_velocity = calculate_imposed_velocity(req.Q, req.T)
    #coeff_a = np.zeros((5,3,4))               # ret = coeff_a [joint][path_segment][coeff of t]
    t = req.T
    coeff_a = [] 
    print("time : " , t)

    

    for j in range(5):
        tmp2 = PathCoefficient_2d()
        for k in range(3):
            A = np.array([ [1,  t[k],  t[k]**2, t[k]**3], [1,  t[k+1],  t[k+1]**2, t[k+1]**3], [0, 1, 2*t[k], 3*(t[k]**2)], [0, 1, 2*t[k+1], 3*(t[k+1]**2)]])
            print("A: /n", A)
            b = np.array([Qn[k].j_value[j], Qn[k+1].j_value[j], Qn_velocity[j,k], Qn_velocity[j, k+1]])
            tmp1 = PathCoefficient_1d()
            tmp1.a = np.linalg.pinv(A) @ b
            tmp2.a_pi.append(tmp1)
        coeff_a.append(tmp2)


    
    return TrajectoryGeneratorResponse(coeff_a)

        

def trajectory_imposed_vel_generator_server():
    rospy.init_node('trajectory_imposed_vel_generator_server')
    srvc = rospy.Service('trajectory_generator_server', TrajectoryGenerator, callback_fn)
    print("trajectory_generator_server (Imposed Velocities) Online!")
    rospy.spin()


if __name__ == "__main__":
    trajectory_imposed_vel_generator_server()
