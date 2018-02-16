# File containing various control functions
import numpy as np
import pdb

def PID(x, y, theta, goal):
# PID for a differential drive robot
    kp_v = 1
    kp_w = 10
    # pdb.set_trace()


    e_d = np.sqrt((x-goal[0])**2 + (y-goal[1])**2)
    e_a = (np.arctan2(goal[1] -y , goal[0] -x)) - theta
    PID_w = kp_w*(e_a)
    PID_v = kp_v*(e_d) - 3*abs(PID_w)

    v = PID_v*(PID_v>=0) + (PID_v<0)*0
    w = PID_w*(abs(PID_w)<=np.pi) + (abs(PID_w)>np.pi)*np.pi

    return v, w, e_d, e_a



def LADRC(X,Y,theta, goal, prev_state,T):

    ''' Linear Advanced Distrubance Rejection Control '''
    fs = 1/T
    K = 2/T

    z1_p = prev_state[0, :]
    z1_pp = prev_state[1, :]
    z2_p = prev_state[2, :]
    z2_pp = prev_state[3, :]
    z3_p = prev_state[4, :]
    z3_pp = prev_state[5, :]
    y_p = prev_state[6, :]
    u_p = prev_state[7, :]
    u_pp = prev_state[8, :]

    e_d = np.sqrt((X-goal[0])**2 + (Y-goal[1])**2)
    e_a = (np.arctan2(goal[1] -Y , goal[0] -X)) - theta
    y = [X,Y,theta]

    # Tunable gains
    b0 = 80
    wc = 3 # < .1*fs
    wo = 15 #5*wc # ~ 5-10 wc

    # PID Gains
    zeta = 0.5
    kd = 2*zeta*wc
    kp = wc**2

    # Estimator Gains
    B_01 = 3*wo
    B_02 = 3*wo**2
    B_03 = wo**3

    # Setpoint
    r= [goal[0], goal[1], np.arctan2(goal[1] -Y , goal[0] -X)]

    z1 = z1_pp + (1/K)*(-B_01*(z1_p+z1_pp) + z2_p + z2_pp + B_01*(y+y_p))
    z2 = z2_pp + (1/K)*(-B_02*(z1+z1_p) + z3_p + z3_pp + b0*(u_p+u_pp) + B_02*(y+y_p))
    z3 = z3_pp + (1/K)*(-B_03*(z1+z1_p) + B_03*(y+y_p))

    # pdb.set_trace()

    u0 = kp*(r-z1) - kd*z2
    u = (-z3+u0)/b0

    cur_state = np.empty_like(prev_state)

    cur_state[0, :] = z1
    cur_state[1, :] = z1_p
    cur_state[2, :] = z2
    cur_state[3, :] = z2_p
    cur_state[4, :] = z3
    cur_state[5, :] = z3_p
    cur_state[6, :] = y
    cur_state[7, :] = u
    cur_state[8, :] = u_p
    cur_state[9,:] = r

    v = np.sqrt(u[0]**2+u[1]**2)#*(u[1]>=0) + (u[1]<0)*0
    w = u[2]

    return v, w, cur_state
