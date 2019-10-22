function [u]= twbr_control2(t,x)
% phi = x(1); theta = x(2); phi_dot = x(3); theta_dot = x(4);
global a1 a2 a3 a4 b1 b2 c1 c2 c3
M = [a3+a2*cos(x(1)), a2*cos(x(1))+a1;
    a2*cos(x(1)), a1];
 
H = [-a2*sin(x(1)).*(x(2)).^2;-a2*sin(x(1)).*(x(2)).^2];
V = -a4*sin(x(1))*[1;0];

calculate_control2