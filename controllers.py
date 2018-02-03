# File containing various control functions
import numpy as np
import pdb

def PID(x, y, theta, goal):
# PID for a differential drive robot
    kp_v = 1;
    kp_w = 10;
    # pdb.set_trace()


    e_d = np.sqrt((x-goal[0])**2 + (y-goal[1])**2);
    e_a = (np.arctan2(goal[1] -y , goal[0] -x)) - theta;
    PID_w = kp_w*(e_a);
    PID_v = kp_v*(e_d) - 3*abs(PID_w) ;

    v = PID_v*(PID_v>=0) + (PID_v<0)*0;
    w = PID_w*(abs(PID_w)<=np.pi) + (abs(PID_w)>np.pi)*np.pi;

    return v, w, e_d, e_a
