## Modeling of Differential Drive Robot Kinematics

import numpy as np
import matplotlib.pyplot as plt
from controllers import PID, LADRC
import pdb

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
ur = 1*(2*np.pi) # right wheel velocity rad/s
ul = 1.1*(2*np.pi) # left wheel velocity rad/s
v_ = (r/2)*(ur+ul) # linear velocity
w_ = (r/L)*(ur-ul) # angular velocity



goal = [-5, 4]

prev_state = np.zeros((10,3));



for k in range(1,n):
    # PID Control
    #v[k], w[k], e_d[k], e_a[k] = PID(x[k-1], y[k-1], theta[k-1], goal)

    #LADRC control
    v[k], w[k], prev_state = LADRC(x[k-1], y[k-1], theta[k-1], goal, prev_state, T);

    # pdb.set_trace()

    #Model
    theta[k] = theta[k-1] + T/2*(w[k] + w[k-1]);
    x[k] = x[k-1] + T/2*(v[k]*np.cos(theta[k]) + v[k-1]*np.cos(theta[k-1]))
    y[k] = y[k-1] + T/2*(v[k]*np.sin(theta[k]) + v[k-1]*np.sin(theta[k-1]))

    if  np.sqrt((x[k]-goal[0])**2 + (y[k]-goal[1])**2) < .1:
       print('Success \n')
       break

    #real time plot
#     figure(1)
#     hold on
#     plot(x[k], y[k], 'bo')
#     plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g')
#     title('Position Map')
#     pause(.1)
#     axis equal


## Plot

t = [dt*i for i in range(k-1)]
plt.figure(1)
plt.plot(x, y, 'bo')
plt.plot(goal[0], goal[1], 'go')
plt.title('Position Map')
plt.xlabel('x')
plt.ylabel('y')

plt.show()

#
#
# figure(2), clf
# subplotfill(3,1,1);
# plot(t,x)
# ylabel('xpos')
# subplotfill(3,1,2);
# plot(t,y)
# ylabel('ypos')
# subplotfill(3,1,3);
# plot(t, theta)
# ylabel('theta')
# xlabel('time')
#
# figure(3), clf
# subplotfill(2,1,1);
# plot(t,E(1, :))
# ylabel('Pos Error')
# subplotfill(2,1,2);
# plot(t,E(2,:))
# ylabel('Theta Error')
#
# figure(4), clf
# subplotfill(2,1,1);
# plot(t,v)
# ylabel('Linear Vel')
# subplotfill(2,1,2);
# plot(t,w)
# ylabel('Angular Vel')
