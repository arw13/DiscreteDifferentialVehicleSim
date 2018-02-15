function [ v, w, e_d, e_a ] = PID(x, y, theta, goal)
% PID for a differential drive robot

    kp_v = 1;
    kp_w = 10;

    e_d = sqrt((x-goal(1))^2 + (y-goal(2))^2);
    e_a = (atan2(goal(2) -y , goal(1) -x)) - theta;
    PID_w = kp_w*(e_a);
    PID_v = kp_v*(e_d) ;%- 3*abs(PID_w) ;
    
    
    v = PID_v*(PID_v>=0) + (PID_v<0)*0;
    w = PID_w*(abs(PID_w)<=pi) + (abs(PID_w)>pi)*pi;
end

