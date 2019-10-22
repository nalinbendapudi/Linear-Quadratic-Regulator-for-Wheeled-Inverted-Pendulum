function [value, isterminal, direction] = guard(t, x)

theta = x(1);

value = (abs(theta) > pi/6);


isterminal = 1;
direction = 0;