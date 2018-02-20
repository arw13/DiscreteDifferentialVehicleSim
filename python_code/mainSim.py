## Modeling of Differential Drive Robot Kinematics

import numpy as np
import matplotlib.pyplot as plt
from controllers import PID, LADRC
import pdb
import time

## Initializations
r = .05 # wheel radius, m
L = .2 #distance between wheels, m
dt = 0.01 # s
T = dt
n = 500 # number of samples

x = np.zeros(n)
y = np.zeros(n)
theta = np.zeros(n)
v = np.zeros(n)
w = np.zeros(n)
e_a = np.zeros(n)
e_d = np.zeros(n)

## Differential Drive relations
ur = (2*np.pi) # right wheel velocity rad/s
ul = (2*np.pi) # left wheel velocity rad/s
v_ = (r/2)*(ur+ul) # linear velocity
w_ = (r/L)*(ur-ul) # angular velocity

vel_r = ((2.0*v(i)) + (w(i)*robot_width))/(wheel_dia);
vel_l = ((2.0*v(i)) - (w(i)*robot_width))/(wheel_dia);


goal = [1, 3]

prev_state = np.mat(np.zeros((10,3)))
state_storage = np.zeros([10,3,n])


for k in range(1,n):
    state_storage[:,:,k] = prev_state

    '''PID Control'''
    # v[k], w[k], e_d[k], e_a[k] = PID(x[k-1], y[k-1], theta[k-1], goal)
    # prev_state[9,0] = goal[0]
    # prev_state[9,1] = goal[1]
    # prev_state[9,2] = np.arctan2(goal[1] -y[k] , goal[0] -x[k])


    '''LADRC control'''
    v[k], w[k], prev_state = LADRC(x[k-1], y[k-1], theta[k-1], goal, prev_state, T);
    w[k] = 20*np.sin(.1*k) +w[k]
    # pdb.set_trace()

    #Model
    theta[k] = theta[k-1] + T/2*(w[k] + w[k-1]);
    x[k] = x[k-1] + T/2*(v[k]*np.cos(theta[k]) + v[k-1]*np.cos(theta[k-1]))
    y[k] = y[k-1] + T/2*(v[k]*np.sin(theta[k]) + v[k-1]*np.sin(theta[k-1]))

    if  np.sqrt((x[k]-goal[0])**2 + (y[k]-goal[1])**2) < .1:
       print('Success \n')
       break




## Plot

t = [dt*i for i in range(k)]
plt.figure(1)
plt.plot(x, y, 'bo')
plt.plot(goal[0], goal[1], 'go')
plt.title('Position Map')
plt.xlabel('x')
plt.ylabel('y')

plt.figure(2)
plt.subplot(311)
plt.plot(t,x[:k])
plt.plot(t,state_storage[9,0,:k])
plt.ylabel('xpos')
plt.subplot(312)
plt.plot(t,y[:k])
plt.plot(t,state_storage[9,1,:k])
plt.ylabel('ypos')
plt.subplot(313)
plt.plot(t,theta[:k])
plt.plot(t,state_storage[9,2,:k])
plt.xlabel('time')
plt.ylabel('theta')


plt.show(block=False)
time.sleep(8)

# plt.close('all')

# figure(5), clf
# subplotfill(3,1,1);
# title('Z1 vs Actual')
# hold on
# plot(t,squeeze(state_storage(1,1,:)))
# plot(t,x, 'r')
# ylabel('x')
# subplotfill(3,1,2);
# hold on
# plot(t,squeeze(state_storage(1,2,:)))
# plot(t,y, 'r')
# ylabel('y')
# subplotfill(3,1,3);
# hold on
# plot(t,squeeze(state_storage(1,3,:)))
# plot(t,theta, 'r')
# ylabel('theta')
# xlabel('time')
#
# figure(6), clf
# subplotfill(3,1,1);
# title('Z1 vs SetPoint')
# hold on
# plot(t,squeeze(state_storage(1,1,:)))
# plot(t,squeeze(state_storage(10,1,:)), 'r')
# ylabel('x')
# subplotfill(3,1,2);
# hold on
# plot(t,squeeze(state_storage(1,2,:)))
# plot(t,squeeze(state_storage(10,2,:)), 'r')
# ylabel('y')
# subplotfill(3,1,3);
# hold on
# plot(t,squeeze(state_storage(1,3,:)))
# plot(t,squeeze(state_storage(10,3,:)), 'r')
# ylabel('theta')
# xlabel('time')
