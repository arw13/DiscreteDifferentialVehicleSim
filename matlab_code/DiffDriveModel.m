%% Modeling of Differential Drive Robot Kinematics
clear 
clc

%% Vheicle Dims and Relatins
r = .05; % wheel radius, m
L = .2; %distance between wheels, m
dt = 0.01; % s
T = dt;

ur = 1*(2*pi); % right wheel velocity rad/s
ul = 1.1*(2*pi);
v_ = (r/2)*(ur+ul);
w_ = (r/L)*(ur-ul);

%% Model States
tspan = ones(1,500);
x = 0;%zeros(1,length(tspan));
y = x;
theta = y;
v = x;
w = v;
E = [x;x];



%% State Storage
prev_state = zeros(10,3);
% state_storage = zeros(10,2,length(tspan));

%% Goal!
goal = [2,2];

figure(1), clf

for k = 2:length(tspan)
    state_storage(:, :, k) = prev_state;
    % Control
    %[v(k), w(k), E(1,k), E(2,k)] = PID(x(k-1), y(k-1), theta(k-1), goal);
    % [v(k), w(k)] = scaled(x(k-1), y(k-1), goal);
    [v(k), w(k), prev_state] = LADRC(x(k-1), y(k-1), theta(k-1), goal, prev_state, T);
   
    n1 = random('norm', 0, 1);
    w(k) = w(k)+0;
        
    %Model
    theta(k) = theta(k-1) + T/2*(w(k) + w(k-1));
    x(k) = x(k-1) + T/2*(v(k)*cos(theta(k)) + v(k-1)*cos(theta(k-1)));
    y(k) = y(k-1) + T/2*(v(k)*sin(theta(k)) + v(k-1)*sin(theta(k-1)));
    
    E(:,k) = [sqrt((x(k)-goal(1))^2 + (y(k)-goal(2))^2); (atan2(goal(2)-y(k),goal(1)-x(k)))-theta(k)];
    
   if  sqrt((x(k)-goal(1))^2 + (y(k)-goal(2))^2) < .05
    fprintf('Success \n')
    break
   end 
   
    %real time plot
%     figure(1)
%     hold on
%     plot(x(k), y(k), 'bo')
%     plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g')
%     title('Position Map')
%     pause(.1) 
%     axis equal 
end
min_err = min(abs(E(:,2:end)), [], 2);
fprintf('%f %f \n' , min_err)
%% Plot
t = dt*(0:k-1);
figure(1), clf
hold on
plot(x, y, 'b.-')
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g')
title('Position Map')
xlabel('x')
ylabel('y')
% for i = 1:length(x);
%     plot( [x(i), x(i)+cos(theta(i))], [y(i), y(i)+ sin(theta(i))], 'b')
% end
axis equal


figure(2), clf
subplotfill(3,1,1);
plot(t,x)
ylabel('xpos')
subplotfill(3,1,2);
plot(t,y)
ylabel('ypos')
subplotfill(3,1,3);
plot(t, theta)
ylabel('theta')
xlabel('time')

figure(3), clf
subplotfill(2,1,1);
plot(t,E(1, :))
ylabel('Pos Error')
subplotfill(2,1,2);
plot(t,E(2,:))
ylabel('Theta Error')

figure(4), clf
subplotfill(2,1,1);
plot(t,v)
ylabel('Linear Vel')
subplotfill(2,1,2);
plot(t,w)
ylabel('Angular Vel')

figure(5), clf
subplotfill(3,1,1);
title('Z1 vs Actual')
hold on 
plot(t,squeeze(state_storage(1,1,:)))
plot(t,x, 'r')
ylabel('x')
subplotfill(3,1,2);
hold on 
plot(t,squeeze(state_storage(1,2,:)))
plot(t,y, 'r')
ylabel('y')
subplotfill(3,1,3);
hold on 
plot(t,squeeze(state_storage(1,3,:)))
plot(t,theta, 'r')
ylabel('theta')
xlabel('time')

figure(6), clf
subplotfill(3,1,1);
title('Z1 vs SetPoint')
hold on 
plot(t,squeeze(state_storage(1,1,:)))
plot(t,squeeze(state_storage(10,1,:)), 'r')
ylabel('x')
subplotfill(3,1,2);
hold on 
plot(t,squeeze(state_storage(1,2,:)))
plot(t,squeeze(state_storage(10,2,:)), 'r')
ylabel('y')
subplotfill(3,1,3);
hold on 
plot(t,squeeze(state_storage(1,3,:)))
plot(t,squeeze(state_storage(10,3,:)), 'r')
ylabel('theta')
xlabel('time')
