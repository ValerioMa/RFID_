function [p_n] = system_dynamic(p,control)
%UPDATE_AGENT Summary of this function goes here
%   Detailed explanation goes here
x = p(1); y = p(2); theta = p(3);
ds = control(1);
alpha = control(2);    
if alpha>0.01
    rad = ds/alpha;
    x_n = x + (rad)*(sin(theta+alpha) - sin(theta));
    y_n = y + (rad)*(-cos(theta+alpha) + cos(theta));
    theta_n = rem((theta + alpha + pi), (2*pi)) - pi;
else
    x_n = x + ds*cos(theta);
    y_n = y + ds*sin(theta);
    theta_n = rem((theta + alpha + pi), (2*pi)) - pi;
end

p_n = [x_n;y_n;theta_n];
end

