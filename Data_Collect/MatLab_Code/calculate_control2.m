if flag2 == 'pfl'
    M11 = a3 + a2*cos(x(1));M12 = a2*cos(x(1))+a1;
    M21 = a2*cos(x(1));M22 = a1;

    alpha = pi/2;
    k_p = 40;
    k_d = 5;
    % v_2 = k_p*(2*alpha/pi*atan(x(3))-x(2))-k_d*(x(4));
    v_2 = k_p*(0-x(1))-k_d*(x(2));
    M21bar = M21-M22/M12*M11;
    V2bar = V(2) - M22/M12*V(1);
    h2bar = H(2) - M22/M12*H(1);
    u = 1/b1*(M21bar*v_2 + V2bar + h2bar);
    U = [0; b1*u];
elseif flag2 == 'lqr'
    A = [0 1 0 0;
         a4/(a3*c1) -(b2*c3)/(a3*c1) 0 (b2*c3)/(a3*c1);
         0 0 0 1;
         -(a2*a4)/(a1*a3*c1) (b2*c2)/(a1*c1) 0 -(b2*c2)/(a1*c1)];
    B = [0 -(b1*c3)/(a3*c1) 0 (b1*c2)/(a1*c1) ]';
    Q = [1 00 0 0
     0 0.01 0 0
     0 0 10 0
     0 0 0 0.08];
    R = 1.0;
    K = lqr(A,B,Q,R);
    u = -K(1)*x(1)-K(2)*x(2)-K(3)*x(3)-K(4)*x(4);
    U = [0; b1*u];
end