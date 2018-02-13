function [ v, w ] = scaled( x, y, goal)
%SATAKE from http://ieeexplore.ieee.org/abstract/document/6385976/
vmin = 1;
vmax = 2;
dmin = 0;
dmax = 2;

dt = sqrt((x-goal(1))^2 + (y-goal(2))^2);

v = vmin*(dt<=dmin) + vmax*(dt>=dmax) +...
    (vmin+((vmax-vmin)*(dt-dmin))/(dmax-dmin))*(dt>dmin && dt<dmax);

Yt = goal(2)-y;
Xt = goal(1)-x;

w = 2*v*Yt/(Xt^2+Yt^2);

end

