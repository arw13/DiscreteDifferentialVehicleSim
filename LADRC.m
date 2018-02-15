function [v, w, cur_state] = LADRC(X,Y,theta, goal, prev_state,T)

% Linear Advanced Distrubance Rejection Control
K = 2/T;

z1_p = prev_state(1, :);
z1_pp = prev_state(2, :);
z2_p = prev_state(3, :);
z2_pp = prev_state(4, :);
z3_p = prev_state(5, :);
z3_pp = prev_state(6, :);
y_p = prev_state(7, :);
u_p = prev_state(8, :);
u_pp = prev_state(9, :);

d = sqrt((X-goal(1))^2 + (Y-goal(2))^2);
e_a = (atan2(goal(2) -Y , goal(1) -X)) - theta;
y = [X, Y,theta];

% Tunable gains
b0 = 50;
wc = 3; % < .1*fs
wo = 15; %5*wc; % ~ 5-10 wc

% PID Gains
zeta = 1;
kd = 2.*zeta.*wc;
kp = wc.^2;

% Estimator Gains
B_01 = 3.*wo; 
B_02 = 3.*wo.^2;
B_03 = wo.^3;

% Setpoint
r= [goal, (atan2(goal(2) -Y , goal(1) -X))];

z1 = z1_p + (1/K)*(-B_01*(z1_p+z1_pp) + z2_p + z2_pp + B_01*(y+y_p));
z2 = z2_p + (1/K)*(-B_02*(z1+z1_p) + z3_p + z3_pp + b0*(u_p+u_pp) + B_02*(y+y_p));
z3 = z3_p + (1/K)*(-B_03*(z1+z1_p) + B_03*(y+y_p));   
    
u0 = kp*(r-z1) - kd*z2;
u = (-z3+u0)/b0;

cur_state(1, :) = z1;
cur_state(2, :) = z1_p;
cur_state(3, :) = z2;
cur_state(4, :) = z2_p;
cur_state(5, :) = z3;
cur_state(6, :) = z3_p;
cur_state(7, :) = y;
cur_state(8, :) = u;
cur_state(9, :) = u_p;
cur_state(10,:) = r;

v = sqrt(u(1)^2+u(2)^2); %(abs(u(1)) - 3*abs(u(2)));%;%*(u(1)>=0) + (u(1)<0)*0;
% v = v*(v>0);
w = u(2);

end